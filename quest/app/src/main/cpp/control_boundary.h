#pragma once
#include "input_mapper.h"
#include "room_environment.h"
#include "sim_settings.h"
#include <chrono>
#include <cmath>
#include <optional>

namespace quest_newton {
using InputTime=std::chrono::steady_clock::time_point;
struct CalibrationEvent {
    ControllerSample controller;
    kinematics::Pose stage_from_world,world_from_base;
    InputTime sampled{};
    std::uint64_t settings_generation=0,reference=0,suspension=0;
};
struct FullInputFrame {
    ControllerSample controller;
    ActionValues actions{};
    kinematics::Pose stage_from_world,world_from_base;
    bool registered=false,menu_open=false;
    std::uint64_t calibration=0,reference=0,suspension=0;
    InputTime sampled{};
    std::optional<CalibrationEvent> calibration_event;
};
inline bool RecentInput(InputTime sampled,InputTime now) {
    const auto age=now-sampled;
    return age>=std::chrono::steady_clock::duration::zero() && age<=std::chrono::milliseconds(100);
}
inline bool TrackedControl(const ControllerSample& s) {
    return s.focused && s.stage_valid && s.pose_active && s.position_valid && s.orientation_valid &&
        s.position_tracked && s.orientation_tracked && s.trigger_active &&
        std::isfinite(s.trigger) && s.trigger>=0 && s.trigger<=1;
}
inline bool ControlsSuppressed(const ControllerSample& s,bool paused,bool menu_open) {
    return paused || menu_open || !TrackedControl(s);
}
inline bool ValidFullInput(const FullInputFrame& input,InputTime now) {
    return RecentInput(input.sampled,now) && input.registered && !input.menu_open && TrackedControl(input.controller);
}
inline bool RoomFrameCurrent(const FullInputFrame& input,InputTime now) {
    return RecentInput(input.sampled,now) && input.registered && input.controller.focused && input.controller.stage_valid;
}
inline bool GripperInputCurrent(const FullInputFrame& batch,const FullInputFrame& fresh,InputTime now) {
    const auto valid=[](InputValue grip){return grip.active && std::isfinite(grip.value) && grip.value>=0 && grip.value<=1;};
    const auto index=static_cast<std::size_t>(SimAction::Gripper);
    return ValidFullInput(batch,now) && ValidFullInput(fresh,now) &&
        batch.reference==fresh.reference && batch.suspension==fresh.suspension && batch.calibration==fresh.calibration &&
        valid(batch.actions[index]) && valid(fresh.actions[index]);
}
inline std::optional<CalibrationEvent> CaptureCalibration(const FullInputFrame& frame,std::uint64_t settings_generation,
                                                         bool manual_pause,bool room_pause,InputTime now) {
    if(manual_pause || room_pause || !ValidFullInput(frame,now) || !frame.controller.calibrate_active ||
       frame.controller.trigger>=.2F)return std::nullopt;
    return CalibrationEvent{frame.controller,frame.stage_from_world,frame.world_from_base,frame.sampled,
                            settings_generation,frame.reference,frame.suspension};
}
inline bool CalibrationIsCurrent(const CalibrationEvent& event,const FullInputFrame& current,std::uint64_t settings_generation,
                                 bool manual_pause,bool room_pause,InputTime now) {
    return !manual_pause && !room_pause && ValidFullInput(current,now) && RecentInput(event.sampled,now) &&
        event.settings_generation==settings_generation && event.reference==current.reference && event.suspension==current.suspension;
}
inline bool RegistrationTracking(bool registered,bool head_valid,bool focused,bool stage_valid,bool controls_suppressed,bool room_pause) {
    // Initial registration is required to acquire room geometry in world space.
    return head_valid && focused && stage_valid && (!registered || (!controls_suppressed && !room_pause));
}
inline bool SubstepBoundaryChanged(const FullInputFrame& batch,std::uint64_t applied_settings,const FullInputFrame& fresh,
                                  std::uint64_t requested_settings,bool commands_pending,InputTime now) {
    return applied_settings!=requested_settings || batch.reference!=fresh.reference || batch.suspension!=fresh.suspension ||
        batch.calibration!=fresh.calibration || commands_pending || (ValidFullInput(batch,now) && !ValidFullInput(fresh,now));
}
} // namespace quest_newton
