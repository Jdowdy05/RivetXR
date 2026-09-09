"""Versioned collision/material choices for one Panda, independent of control.

The original mesh stays the default because the added-pad profiles regressed a
measured force-hold lift trial. Live settings may explicitly select either
experiment; switching models resets the scene. Metadata identifies the actual
compiled model. Material values are simulation choices, not measured rubber data.
"""
from dataclasses import dataclass
from types import MappingProxyType


DEFAULT_PROFILE = 'legacy_mesh_v1'
ENHANCED_PROFILE = 'pad_manipulation_v1'
FINGER_BODIES = ('panda_leftfinger', 'panda_rightfinger')


@dataclass(frozen=True)
class ContactProfile:
    name: str
    pads: bool = False
    enhanced_friction: bool = False

    def solver_options(self):
        if self.enhanced_friction:
            return dict(cone='elliptic', impratio=10., enable_multiccd=True)
        return {}


PROFILES = MappingProxyType({
    'legacy_mesh_v1': ContactProfile('legacy_mesh_v1'),
    'five_pads_v1': ContactProfile('five_pads_v1', pads=True),
    ENHANCED_PROFILE: ContactProfile(ENHANCED_PROFILE, pads=True, enhanced_friction=True),
})


def resolve_profile(name):
    if not isinstance(name, str) or name not in PROFILES:
        raise ValueError(f'Unknown contact profile: {name!r}')
    return PROFILES[name]


def apply_profile(builder, profile):
    """Add collision-only pads, then set finger materials before finalization."""
    if profile.pads:
        from .finger_pads import add_finger_pads
        add_finger_pads(builder)
    if profile.enhanced_friction:
        bodies = {builder.body_label.index(name) for name in FINGER_BODIES}
        condim = builder.custom_attributes['mujoco:condim']
        if condim.values is None:
            condim.values = {}
        for shape, body in enumerate(builder.shape_body):
            if body in bodies:
                builder.shape_material_mu[shape] = 1.5
                builder.shape_material_mu_torsional[shape] = .005
                condim.values[shape] = 4


def native_metadata(session):
    """Report effective compiled contact options, not just requested values."""
    profile = session._contact_profile
    mj, m = session.solver._mujoco, session.solver.mj_model
    fingers = []
    for joint_name, body_name in zip(('panda_finger_joint1', 'panda_finger_joint2'), FINGER_BODIES):
        joint = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, joint_name)
        if joint < 0:
            raise ValueError('Finger joint absent from compiled contact model')
        indices = session.np.flatnonzero(m.geom_bodyid == m.jnt_bodyid[joint])
        types = m.geom_type[indices]
        fingers.append(dict(body=body_name, geom_count=len(indices),
                            box_count=int(sum(types == mj.mjtGeom.mjGEOM_BOX)),
                            mesh_count=int(sum(types == mj.mjtGeom.mjGEOM_MESH)),
                            contact_dimensions=sorted(set(map(int, m.geom_condim[indices]))),
                            friction=m.geom_friction[indices].tolist()))
    return dict(profile=profile.name, added_pads_per_finger=5 if profile.pads else 0,
                cone='elliptic' if m.opt.cone == mj.mjtCone.mjCONE_ELLIPTIC else 'pyramidal',
                impratio=float(m.opt.impratio),
                multiccd=not bool(int(m.opt.disableflags) & int(mj.mjtDisableBit.mjDSBL_MULTICCD)),
                fingers=fingers)
