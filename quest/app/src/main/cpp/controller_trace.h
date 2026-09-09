#pragma once

#include "input_mapper.h"

#include <cstdint>

namespace quest_newton::verification {

struct RecordedControllerSample {
    kinematics::Pose stage_from_grip;
    float trigger = 0;
    std::uint32_t flags = 0;
};

// Serialized flags are independent of OpenXR enum values. Assign every field
// explicitly so omitted validity/activity bits fail closed during replay.
inline ControllerSample ToControllerSample(const RecordedControllerSample& recorded) {
    ControllerSample sample;
    sample.stage_from_grip = recorded.stage_from_grip;
    sample.trigger = recorded.trigger;
    sample.focused = (recorded.flags & (1U << 0)) != 0;
    sample.stage_valid = (recorded.flags & (1U << 1)) != 0;
    sample.pose_active = (recorded.flags & (1U << 2)) != 0;
    sample.trigger_active = (recorded.flags & (1U << 3)) != 0;
    sample.calibrate_active = (recorded.flags & (1U << 4)) != 0;
    sample.position_valid = (recorded.flags & (1U << 5)) != 0;
    sample.orientation_valid = (recorded.flags & (1U << 6)) != 0;
    sample.position_tracked = (recorded.flags & (1U << 7)) != 0;
    sample.orientation_tracked = (recorded.flags & (1U << 8)) != 0;
    sample.calibrate_pressed = (recorded.flags & (1U << 9)) != 0;
    return sample;
}

} // namespace quest_newton::verification
