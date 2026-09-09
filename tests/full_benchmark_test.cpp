#include "full_benchmark.h"
#include "simulation_clock.h"
#include <cstdio>
#include <stdexcept>

namespace {
void Check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
}
int main() {
    using namespace quest_newton;
    try {
        verification::LaunchOptions options;
        std::string error;
        for (int hz : {50, 100, 200, 250, 300, 350, 400, 500, 1000, 2000}) {
            options.benchmark_hz = hz;
            const auto settings = BenchmarkSettings(options);
            Check(ValidateSettings(settings, error), "benchmark settings rejected");
            Check(settings.gripper_speed_mps==0&&!settings.gripper_force_hold,
                  "benchmark must bypass live gripper postprocessing defaults");
            Check(settings.contact_profile==ContactProfile::OriginalMesh,"benchmark must keep Original contact model");
            Check(std::abs(settings.physics_dt * hz - 1) < 1e-10, "physics Hz not preserved");
            Check(settings.render_interval == settings.control_decimation && !settings.floating_base &&
                !settings.follow_height && settings.gravity_scale == 1, "benchmark contract changed");
            SimulationClock clock;
            Check(clock.Configure(settings, 0), "clock configure failed");
            clock.Reset(0);
            unsigned steps = 0, controls = 0, publications = 0;
            for (int ms = 1; ms <= 1000; ++ms) {
                for (unsigned n = clock.Accumulate(ms / 1000.0, false); n > 0; --n) {
                    const auto tick = clock.Advance(); ++steps;
                    controls += tick.control_due; publications += tick.publish_due;
                }
            }
            Check(steps == static_cast<unsigned>(hz), "ideal millisecond pacing lost steps");
            Check(publications == steps / settings.render_interval, "publication cadence wrong");
            Check(controls == (steps + settings.control_decimation - 1) / settings.control_decimation, "control cadence wrong");
            Check(clock.DroppedWallSeconds() == 0, "ideal pacing dropped wall time");
            Check(clock.Accumulate(2, false) == 4 && clock.DroppedWallSeconds() > .9, "overload did not preserve production drop policy");
            clock.Reset(10);
            Check(clock.SimulationTime() == 0 && clock.DroppedWallSeconds() == 0 && clock.Accumulate(10, false) == 0,
                "warmup reset retained time or catchup");
        }
        options.benchmark_hz = 250;
        Check(BenchmarkSettings(options).control_decimation == 3, "250 Hz must report its rounded 83.33 Hz control cadence");
        const auto rest = BenchmarkWorkload(5, false);
        kinematics::Pose home;
        Check(kinematics::ForwardKinematics(kinematics::kHome, home), "home FK failed");
        Check(rest.hand.position == home.position && rest.hand.rotation == home.rotation && rest.gripper == 0 &&
            rest.base_height_offset == 0, "rest workload moves");
        auto joints = kinematics::kHome;
        double max_delta = 0, max_gripper = 0;
        for (int tick = 0; tick < 800; ++tick) {
            const auto target = BenchmarkWorkload(tick / 100.0, true);
            const auto repeat = BenchmarkWorkload(tick / 100.0, true);
            Check(target.hand.position == repeat.hand.position && target.gripper == repeat.gripper, "workload not deterministic");
            Check(target.gripper >= 0 && target.gripper <= 1 && std::abs(target.base_height_offset) <= .020001, "waveform unbounded");
            for (std::size_t axis = 0; axis < 3; ++axis)
                Check(std::abs(target.hand.position[axis] - home.position[axis]) <= .040001, "hand motion too large");
            const auto result = kinematics::SolveIK(joints, target.hand);
            Check(result.valid && kinematics::WithinJointLimits(result.joints), "moving IK invalid");
            joints = result.joints;
            for (std::size_t j = 0; j < joints.size(); ++j) max_delta = std::max(max_delta, static_cast<double>(std::abs(joints[j] - kinematics::kHome[j])));
            max_gripper = std::max(max_gripper, static_cast<double>(target.gripper));
        }
        Check(max_delta > .05 && max_gripper > .99, "motion workload did not exercise arm/gripper");
        std::puts("benchmark workload, rates and production pacing passed");
        return 0;
    } catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
}
