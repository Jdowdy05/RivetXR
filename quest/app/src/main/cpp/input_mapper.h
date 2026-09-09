#pragma once

#include "quest_newton/franka_kinematics.h"

#include <string_view>

namespace quest_newton {

// All poses are metres and XYZW quaternions. The adapter samples grip in STAGE
// at predicted display time; head/view pose deliberately is not an input.
struct ControllerSample {
    kinematics::Pose stage_from_grip;
    bool focused = false;
    bool stage_valid = false;
    bool pose_active = false;
    bool trigger_active = false;
    bool calibrate_active = false;
    bool position_valid = false;
    bool orientation_valid = false;
    bool position_tracked = false;
    bool orientation_tracked = false;
    bool calibrate_pressed = false;
    float trigger = 0;
};

struct MappedTarget {
    kinematics::JointVector joints = kinematics::kHome;
    kinematics::Pose robot_base_from_stage;
    kinematics::Pose robot_base_from_grip;
    bool calibrated = false;
    bool engaged = false;
    std::string_view reason = "uncalibrated";
};

class InputMapper {
public:
    const MappedTarget& Update(const ControllerSample& sample);
    const MappedTarget& Current() const { return current_; }
    // Call when the STAGE reference changes. Held targets survive; a new
    // calibration press and a released trigger are required before control.
    void InvalidateCalibration();

private:
    const MappedTarget& Disengage(std::string_view reason);
    MappedTarget current_;
    bool release_observed_ = false;
    bool calibrate_was_pressed_ = false;
};

}  // namespace quest_newton
