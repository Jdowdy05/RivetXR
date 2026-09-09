#pragma once
#include "launch_options.h"
#include "sim_settings.h"
#include "quest_newton/franka_kinematics.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace quest_newton {
inline SimSettings BenchmarkSettings(const verification::LaunchOptions& options) {
    SimSettings settings;
    settings.physics_dt = 1.0 / options.benchmark_hz;
    settings.control_decimation = settings.render_interval = static_cast<std::uint32_t>(
        std::max(1L, std::lround(options.benchmark_hz / 100.0)));
    settings.floating_base = false;
    settings.follow_height = false;
    // Preserve the historical benchmark's direct finger-target commands.
    settings.gripper_force_hold = false;
    settings.gripper_speed_mps = 0;
    settings.contact_profile = ContactProfile::OriginalMesh;
    settings.shoulder_offset_m = .5F;
    return settings;
}
struct BenchmarkTarget {
    kinematics::Pose hand;
    float gripper = 0;
    float base_height_offset = 0;
};
// Simulation-time waveforms are deterministic across CPU speeds. No input or
// state clipping is applied; SolveIK retains its production target limits.
inline BenchmarkTarget BenchmarkWorkload(double simulation_seconds, bool motion) {
    BenchmarkTarget target;
    kinematics::ForwardKinematics(kinematics::kHome, target.hand);
    if (!motion) return target;
    const double phase = 2 * std::numbers::pi * simulation_seconds / 4.0;
    target.hand.position[0] += static_cast<float>(.03 * std::sin(phase));
    target.hand.position[1] += static_cast<float>(.04 * std::sin(phase * .5));
    target.hand.position[2] += static_cast<float>(.02 * std::sin(phase * .75));
    target.gripper = static_cast<float>(.5 - .5 * std::cos(phase));
    target.base_height_offset = static_cast<float>(.02 * std::sin(phase * .5));
    return target;
}
} // namespace quest_newton
