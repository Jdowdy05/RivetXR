#pragma once

#include "room_environment.h"
#include <openxr/openxr.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace quest_newton {
// XR-thread only. The caller owns permission prompts and pauses physics whenever
// collisions are requested but Ready() is false. Shutdown before xrDestroySession.
class RoomScene {
public:
    using ClockNow = std::function<std::chrono::steady_clock::time_point()>;
    // An empty clock uses steady_clock; injection keeps timeout checks sleep-free.
    explicit RoomScene(ClockNow now = {});
    ~RoomScene();
    RoomScene(const RoomScene&) = delete;
    RoomScene& operator=(const RoomScene&) = delete;
    static std::vector<const char*> Extensions();
    bool Init(XrInstance instance, XrSession session);
    void Shutdown();
    void SetEnabled(bool enabled);
    // Automatic reference recovery passes false and cannot clear a health latch.
    // False means an explicit user Refresh/Scan (or Off-On) is still required.
    bool Refresh(bool explicit_request = true);
    // True means queued/already active, not that OS capture has begun. On false,
    // retain Status() as an operator diagnostic independently of room readiness.
    bool RequestCapture();
    // A changed STAGE or registration invalidates the captured geometry. The
    // caller may request automatic Refresh(false) after establishing the new
    // frame; an existing health latch always retains its user-recovery requirement.
    void Invalidate();
    void OnEvent(const XrEventDataBaseHeader* event);
    void Update(XrTime time, XrSpace stage,
                const kinematics::Pose& stage_from_world,
                const kinematics::Pose& stage_head,
                bool tracking_valid, bool permission_granted);
    const RoomEnvironment& Snapshot() const;
    // Applied anchor poses are checked in bounded sweeps. Drift, localization
    // loss or expired verification latches false until Refresh/Scan; the last
    // snapshot and revision stay unchanged so physics/rendering can pause it.
    // This does not track furniture moved without an updated Scene capture.
    bool Ready() const;
    const std::string& Status() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace quest_newton
