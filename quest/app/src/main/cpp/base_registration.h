#pragma once
#include "sim_settings.h"
#include "quest_newton/franka_kinematics.h"

namespace quest_newton {
kinematics::Pose HandOffsetPose(const std::array<float,3>& degrees);
class BaseRegistration {
public:
    bool Update(const kinematics::Pose& stage_from_head,bool tracking,bool clutch,const SimSettings& settings);
    void Invalidate() {valid_=false;tracked_=false;}
    bool Valid() const {return valid_;}
    const kinematics::Pose& StageFromWorld() const {return stage_from_world_;}
    const kinematics::Pose& WorldFromBase() const {return world_from_base_;}
    kinematics::Pose StageFromBase() const {return kinematics::Compose(stage_from_world_,world_from_base_);}
private:
    kinematics::Pose stage_from_world_,world_from_base_;
    bool valid_=false,tracked_=false,last_clutch_=false,last_follow_=true,last_floating_=false;
    float last_head_height_=0,last_shoulder_offset_=.25F;
};
} // namespace quest_newton
