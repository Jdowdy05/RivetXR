#pragma once
#include "input_mapper.h"

namespace quest_newton {
// Calibration recenters position and restores the configured default palm
// orientation. The independent physics base stays gravity aligned. Worker owned.
class RegisteredInputMapper {
public:
    const MappedTarget& Update(const ControllerSample& sample,const kinematics::Pose& stage_from_base,
                              const kinematics::Pose& palm_offset);
    const MappedTarget& Current() const{return current_;}
    void RequestCalibration(){calibration_requested_=true;}
    void Reset(){*this=RegisteredInputMapper{};}
    void Invalidate(){current_.calibrated=false;Disengage("recalibrate after reference change");}
private:
    const MappedTarget& Disengage(std::string_view reason);
    MappedTarget current_;
    kinematics::Pose grip_from_registered_palm_;
    kinematics::Quaternion calibration_grip_rotation_{0,0,0,1};
    kinematics::Quaternion neutral_palm_stage_rotation_{0,0,0,1};
    bool release_observed_=false,calibrate_down_=false,calibration_requested_=false;
};
} // namespace quest_newton
