#include "input_mapper.h"

#include <algorithm>
#include <cmath>

namespace quest_newton {
namespace kin = kinematics;
namespace {
constexpr float kReleaseThreshold = .2F;
constexpr float kEngageThreshold = .5F;
}

const MappedTarget& InputMapper::Disengage(std::string_view reason) {
    current_.engaged = false;
    current_.reason = reason;
    release_observed_ = false;
    // A failed/inactive sample cannot prove a button release. Reacquisition
    // while A remains held must not silently recalibrate the base.
    calibrate_was_pressed_ = true;
    return current_;
}

void InputMapper::InvalidateCalibration() {
    current_.calibrated = false;
    Disengage("uncalibrated");
}

const MappedTarget& InputMapper::Update(const ControllerSample& sample) {
    const bool calibration_press = sample.calibrate_active &&
        sample.calibrate_pressed && !calibrate_was_pressed_;
    calibrate_was_pressed_ = !sample.calibrate_active || sample.calibrate_pressed;

    if (!sample.focused) return Disengage("unfocused");
    if (!sample.stage_valid) return Disengage("stage unavailable");
    if (!sample.pose_active || !sample.trigger_active) return Disengage("inactive input");
    if (!sample.position_valid || !sample.orientation_valid ||
        !sample.position_tracked || !sample.orientation_tracked) {
        return Disengage("tracking unavailable");
    }
    if (!std::isfinite(sample.trigger) || sample.trigger < 0 || sample.trigger > 1) {
        return Disengage("invalid trigger");
    }
    kin::Pose stage_from_grip;
    if (!kin::NormalizePose(sample.stage_from_grip, stage_from_grip)) {
        return Disengage("invalid grip pose");
    }

    const bool released = sample.trigger < kReleaseThreshold;
    if (calibration_press && released) {
        kin::Pose robot_base_from_hand;
        if (!kin::ForwardKinematics(current_.joints, robot_base_from_hand)) {
            return Disengage("invalid held target");
        }
        kin::Pose robot_base_from_stage;
        if (!kin::NormalizePose(kin::Compose(robot_base_from_hand,
                kin::Inverse(stage_from_grip)), robot_base_from_stage)) {
            return Disengage("invalid calibration");
        }
        current_.robot_base_from_stage = robot_base_from_stage;
        current_.robot_base_from_grip = robot_base_from_hand;
        current_.calibrated = true;
    }

    if (!current_.calibrated) {
        current_.engaged = false;
        current_.reason = "uncalibrated";
        release_observed_ = false;
        return current_;
    }
    if (released) {
        current_.engaged = false;
        current_.reason = "trigger released";
        release_observed_ = true;
        return current_;
    }
    if (!current_.engaged) {
        if (!release_observed_) {
            current_.reason = "release trigger to rearm";
            return current_;
        }
        if (sample.trigger <= kEngageThreshold) {
            current_.reason = "trigger released";
            return current_;
        }
    }

    kin::Pose robot_base_from_grip;
    if (!kin::NormalizePose(kin::Compose(current_.robot_base_from_stage,
            stage_from_grip), robot_base_from_grip)) {
        return Disengage("invalid mapped pose");
    }
    const auto solved = kin::SolveIK(current_.joints, robot_base_from_grip);
    if (!solved.valid || !std::isfinite(solved.position_error) ||
        !std::isfinite(solved.orientation_error) ||
        !std::all_of(solved.joints.begin(), solved.joints.end(),
                     [](float joint) { return std::isfinite(joint); })) {
        return Disengage("invalid inverse kinematics");
    }
    current_.joints = solved.joints;
    current_.robot_base_from_grip = robot_base_from_grip;
    current_.engaged = true;
    current_.reason = solved.converged ? "engaged" : "engaged bounded IK";
    return current_;
}

}  // namespace quest_newton
