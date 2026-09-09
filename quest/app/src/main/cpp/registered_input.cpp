#include "registered_input.h"
#include <cmath>

namespace quest_newton {
namespace kin=kinematics;
const MappedTarget& RegisteredInputMapper::Disengage(std::string_view reason) {
    current_.engaged=false;current_.reason=reason;release_observed_=false;calibrate_down_=true;return current_;
}
const MappedTarget& RegisteredInputMapper::Update(const ControllerSample& sample,const kin::Pose& base,const kin::Pose& offset) {
    const bool pressed=sample.calibrate_active && (calibration_requested_ || (sample.calibrate_pressed && !calibrate_down_));
    calibration_requested_=false;
    calibrate_down_=!sample.calibrate_active || sample.calibrate_pressed;
    if(!sample.focused || !sample.stage_valid) return Disengage("unfocused or reference unavailable");
    if(!sample.pose_active || !sample.trigger_active || !sample.position_valid || !sample.orientation_valid ||
       !sample.position_tracked || !sample.orientation_tracked) return Disengage("tracking unavailable");
    if(!std::isfinite(sample.trigger) || sample.trigger<0 || sample.trigger>1) return Disengage("invalid trigger");
    kin::Pose grip,stage_base,palm_offset;
    if(!kin::NormalizePose(sample.stage_from_grip,grip) || !kin::NormalizePose(base,stage_base) ||
       !kin::NormalizePose(offset,palm_offset)) return Disengage("invalid registration pose");
    current_.robot_base_from_stage=kin::Inverse(stage_base);
    const bool released=sample.trigger<.2F;
    if(pressed && released) {
        kin::Pose palm,neutral;
        if(!kin::ForwardKinematics(current_.joints,palm) || !kin::ForwardKinematics(kin::kHome,neutral))
            return Disengage("invalid held joints");
        grip_from_registered_palm_=kin::Compose(kin::Inverse(grip),kin::Compose(stage_base,palm));
        calibration_grip_rotation_=grip.rotation;
        neutral_palm_stage_rotation_=kin::Compose(stage_base,neutral).rotation;
        current_.calibrated=true;
    }
    if(!current_.calibrated) {current_.engaged=false;current_.reason="press calibration with trigger released";return current_;}
    if(released) {release_observed_=true;current_.engaged=false;current_.reason="trigger released";return current_;}
    if(!current_.engaged && (!release_observed_ || sample.trigger<=.5F)) {
        current_.reason="release trigger to rearm";return current_;
    }
    auto stage_palm=kin::Compose(grip,grip_from_registered_palm_);
    // Apply the default about the controller's calibration axes. Reusing the
    // already offset held orientation here would accumulate the offset on A.
    const kin::Pose current_grip_rotation{{0,0,0},grip.rotation};
    const kin::Pose calibration_grip_rotation{{0,0,0},calibration_grip_rotation_};
    const kin::Pose neutral_rotation{{0,0,0},neutral_palm_stage_rotation_};
    stage_palm.rotation=kin::Compose(kin::Compose(current_grip_rotation,palm_offset),
        kin::Compose(kin::Inverse(calibration_grip_rotation),neutral_rotation)).rotation;
    const auto target=kin::Compose(current_.robot_base_from_stage,stage_palm);
    const auto solved=kin::SolveIK(current_.joints,target);
    if(!solved.valid || !std::isfinite(solved.position_error) || !std::isfinite(solved.orientation_error) ||
       !kin::WithinJointLimits(solved.joints)) return Disengage("invalid IK solution");
    current_.joints=solved.joints;current_.robot_base_from_grip=target;current_.engaged=true;
    current_.reason=solved.converged?"engaged":"engaged bounded IK";return current_;
}
} // namespace quest_newton
