#pragma once

#include "quest_newton/franka_kinematics.h"

#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>

namespace quest_newton {
struct JointTargetSnapshot {
    kinematics::JointVector joints = kinematics::kHome;
    std::chrono::steady_clock::time_point published_at{};
    std::uint64_t generation = 0;
    bool engaged = false;
};

// Render-to-physics intent only. Disengagement freezes the last applied joint
// target; Newton continues prediction toward that target. There is no actuation.
class JointTargetMailbox {
  public:
    using Clock = std::chrono::steady_clock;
    bool Publish(const kinematics::JointVector& joints, bool engaged,
                 Clock::time_point now = Clock::now()) {
        if (!kinematics::WithinJointLimits(joints)) return false;
        const std::lock_guard lock(mutex_);
        if (snapshot_.generation == std::numeric_limits<std::uint64_t>::max()) return false;
        snapshot_ = {joints, now, snapshot_.generation + 1, engaged};
        return true;
    }
    JointTargetSnapshot Read() const {
        auto copy = Copy();
        // Sample after copying: a simultaneous publication cannot appear to
        // come from the future solely because time was sampled before locking.
        return CheckFreshness(copy, Clock::now());
    }
    JointTargetSnapshot Read(Clock::time_point now) const {
        return CheckFreshness(Copy(), now);
    }
  private:
    JointTargetSnapshot Copy() const {
        const std::lock_guard lock(mutex_);
        return snapshot_;
    }
    static JointTargetSnapshot CheckFreshness(JointTargetSnapshot copy, Clock::time_point now) {
        if (now < copy.published_at || now - copy.published_at > std::chrono::milliseconds(100)) {
            copy.engaged = false;
        }
        return copy;
    }
    mutable std::mutex mutex_;
    JointTargetSnapshot snapshot_;
};
} // namespace quest_newton
