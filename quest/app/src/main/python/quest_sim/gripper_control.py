"""Bounded Franka finger position targets with optional contact-force feedback.

This module has no physics/runtime dependencies and never writes measured state.
The caller supplies fresh closing-opposed force magnitudes from the latest
physics solve, or None when feedback is unavailable. It is not a grasp detector,
attachment, hard force clamp, or replacement for the model's actuator limits.
"""
from dataclasses import dataclass
import math
from numbers import Real


OPEN_POSITION_M = .04
RELEASE_THRESHOLD = .02
CONTACT_THRESHOLD_N = 1e-6
# Position-setpoint admittance after acquisition. Actual contact force may
# overshoot because of inertia, contact compliance, and delayed physics response.
FORCE_ADMITTANCE_M_PER_NS = .002


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return float(value)


def _pair(values, label, nonnegative=False):
    try:
        if len(values) != 2:
            raise ValueError(f'{label} must contain two values')
        result = tuple(_finite(value, label) for value in values)
    except TypeError:
        raise ValueError(f'{label} must contain two values') from None
    if nonnegative and min(result) < 0:
        raise ValueError(f'{label} must be nonnegative')
    return result


def _target_bound(value):
    # Only commanded targets are bounded. The measured simulation arrays are
    # never clipped or modified, including at reset after soft joint violation.
    return min(OPEN_POSITION_M, max(0., value))


@dataclass(frozen=True)
class GripperSettings:
    force_hold: bool = False
    max_speed_m_s: float = .05
    max_force_n: float = 5.

    def __post_init__(self):
        if type(self.force_hold) is not bool:
            raise ValueError('force_hold must be boolean')
        speed = _finite(self.max_speed_m_s, 'max_speed_m_s')
        force = _finite(self.max_force_n, 'max_force_n')
        if not .005 <= speed <= .2:
            raise ValueError('max_speed_m_s must be in [.005,.2]')
        if not .5 <= force <= 20.:
            raise ValueError('max_force_n must be in [.5,20]')


class GripperController:
    """Physics-step-owned target controller; all retained state is plain values.

    A new/reset scene calls reset(measured_q). A transaction preserving the
    current command calls reset(measured_q, target_q=current_target), then
    invalidate_feedback(). No mutable array or feedback history is retained;
    copy.deepcopy is safe if the caller instead needs an independent state copy.

    Suppression or missing force feedback retires acquisition and freezes the
    position setpoint. This is not a hard force hold during suppression: the
    existing physical servo may settle against its frozen setpoint. Resume must
    be authorized by the caller's existing input release/rearm guards.
    """
    def __init__(self, measured_q=(OPEN_POSITION_M, OPEN_POSITION_M)):
        self.reset(measured_q)

    def reset(self, measured_q, target_q=None):
        measured = _pair(measured_q, 'measured_q')
        seed = measured if target_q is None else _pair(target_q, 'target_q')
        target = tuple(_target_bound(value) for value in seed)
        self._target = target
        self._measured = measured
        self._acquired = False
        self._force_hold = False
        self._desired_force = 0.
        self._measured_force = None
        self._phase = 'reset'

    def invalidate_feedback(self):
        """Retire feedback after topology/base changes without moving targets."""
        self._acquired = False
        self._measured_force = None
        self._phase = 'feedback_invalidated'

    def step(self, dt, measured_q, opposing_forces, requested_grip, allowed, settings):
        """Return two metre-valued targets without changing physical state.

        opposing_forces: fresh (left,right) nonnegative force magnitudes, or
        None. A fresh zero tuple means no currently loaded opposing contact.
        Use the maximum per-finger sum so a one-sided contact also limits closing.
        Contact regulation remains acquired across fresh zero-force samples;
        further closing is then limited to the bounded admittance rate while
        the user continues to request it. There is no remembered stale force.

        Release in force mode opens even if force feedback is unavailable.
        Off mode does not require force feedback. Every normal step moves each
        target by at most max_speed_m_s*dt. Toggling modes never rebases targets.
        """
        # Validate before changing anything; failure leaves controller state
        # unchanged so the caller can handle the invalid input consistently.
        delta_time = _finite(dt, 'dt')
        if not .0005 <= delta_time <= .02:
            raise ValueError('dt must be in [.0005,.02]')
        measured = _pair(measured_q, 'measured_q')
        forces = None if opposing_forces is None else _pair(opposing_forces, 'opposing_forces', True)
        grip = _finite(requested_grip, 'requested_grip')
        if not 0. <= grip <= 1.:
            raise ValueError('requested_grip must be in [0,1]')
        if type(allowed) is not bool:
            raise ValueError('allowed must be boolean')
        if not isinstance(settings, GripperSettings):
            raise ValueError('validated GripperSettings required')

        if settings.force_hold != self._force_hold:
            self._acquired = False
        self._force_hold = settings.force_hold
        self._measured = measured
        self._desired_force = grip * settings.max_force_n if settings.force_hold and grip > RELEASE_THRESHOLD else 0.
        self._measured_force = None
        maximum_change = settings.max_speed_m_s * delta_time

        if not allowed:
            self._acquired = False
            self._phase = 'suppressed'
            return self._target
        if not settings.force_hold:
            self._acquired = False
            self._phase = 'position'
            goal = OPEN_POSITION_M * (1. - grip)
            self._target = tuple(_target_bound(value + min(maximum_change, max(-maximum_change, goal-value)))
                                 for value in self._target)
        elif grip <= RELEASE_THRESHOLD:
            self._acquired = False
            self._phase = 'opening'
            self._target = tuple(min(OPEN_POSITION_M, value + maximum_change) for value in self._target)
        elif forces is None:
            self._acquired = False
            self._phase = 'feedback_unavailable'
        else:
            self._measured_force = max(forces)
            if self._measured_force > CONTACT_THRESHOLD_N:
                self._acquired = True
            if self._acquired:
                speed = FORCE_ADMITTANCE_M_PER_NS * (self._measured_force - self._desired_force)
                speed = min(settings.max_speed_m_s, max(-settings.max_speed_m_s, speed))
                self._phase = 'force_hold'
            else:
                speed = -settings.max_speed_m_s
                self._phase = 'closing_until_contact'
            self._target = tuple(_target_bound(value + speed * delta_time) for value in self._target)
        return self._target

    def diagnostics(self):
        return dict(phase=self._phase, target_q=self._target, measured_q=self._measured,
                    force_hold=self._force_hold, desired_force_n=self._desired_force,
                    measured_max_force_n=self._measured_force, contact_acquired=self._acquired)
