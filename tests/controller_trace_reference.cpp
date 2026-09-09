// Production mapper/IK reference. Newton state is generated separately by
// loading the actual captured CPU graph, never by approximating its dynamics.
#include "controller_trace.h"
#include "controller_trace_data.h"
#include "verification_state_limits.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>

namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// Deliberately independent of both the Python phase builder and golden states.
bool ExpectedEngaged(std::size_t index) {
    return (index >= 30 && index < 350) || (index >= 380 && index < 400) ||
           (index >= 440 && index < 600) || (index >= 640 && index < 850) ||
           (index >= 900 && index < 980);
}

void CheckFlagConversion() {
    using quest_newton::verification::RecordedControllerSample;
    using quest_newton::verification::ToControllerSample;
    for (unsigned bit = 0; bit < 10; ++bit) {
        RecordedControllerSample recorded{{{1.F, 2.F, 3.F}, {0.F, 0.F, 0.F, 1.F}}, .8F, 1U << bit};
        const auto sample = ToControllerSample(recorded);
        const std::array<bool, 10> actual{sample.focused, sample.stage_valid,
            sample.pose_active, sample.trigger_active, sample.calibrate_active,
            sample.position_valid, sample.orientation_valid, sample.position_tracked,
            sample.orientation_tracked, sample.calibrate_pressed};
        for (unsigned field = 0; field < actual.size(); ++field)
            Require(actual[field] == (field == bit), "recorded flag mapping differs");
        Require(sample.trigger == .8F && sample.stage_from_grip.position == recorded.stage_from_grip.position &&
                sample.stage_from_grip.rotation == recorded.stage_from_grip.rotation, "recorded pose/trigger differs");
    }
}
}

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "usage: controller_trace_reference OUTPUT_JSON");
        CheckFlagConversion();
        namespace trace = quest_newton::generated_trace;
        namespace kin = quest_newton::kinematics;
        static_assert(trace::kSamples.size() == 1000 && trace::kSubstepsPerSample == 10);
        std::ofstream output(argv[1], std::ios::binary);
        Require(output.good(), "cannot open reference output");
        output.imbue(std::locale::classic());
        output << std::setprecision(std::numeric_limits<float>::max_digits10) << std::boolalpha;
        output << "{\"schema_version\":1,\"trace_sha256\":\"" << trace::kTraceSha256
               << "\",\"physics_manifest_sha256\":\"" << trace::kPhysicsManifestSha256 << "\",\"state_limit_policy\":";
        quest_newton::verification::WriteStateLimitPolicy(output);
        output << ",\"samples\":[";
        quest_newton::InputMapper mapper;
        auto previous = kin::kHome;
        auto low = previous;
        auto high = previous;
        float max_delta = 0;
        for (std::size_t index = 0; index < trace::kSamples.size(); ++index) {
            const auto& target = mapper.Update(quest_newton::verification::ToControllerSample(trace::kSamples[index]));
            Require(target.engaged == ExpectedEngaged(index), "engagement/rearm expectation failed");
            Require(target.calibrated == (index >= 1), "calibration expectation failed");
            if (index == 1) {
                const auto stage_from_base = kin::Inverse(target.robot_base_from_stage);
                const kin::Pose provisional{{0.F, 0.F, -1.2F}, {-.5F, .5F, .5F, .5F}};
                for (std::size_t axis = 0; axis < 3; ++axis)
                    Require(std::abs(stage_from_base.position[axis] - provisional.position[axis]) < 1e-5F,
                            "independent source FK did not preserve initial base placement");
                float dot = 0.F;
                for (std::size_t component = 0; component < 4; ++component)
                    dot += stage_from_base.rotation[component] * provisional.rotation[component];
                Require(std::abs(dot) > .999999F, "independent source FK initial orientation differs");
            }
            Require(kin::WithinJointLimits(target.joints), "target outside joint limits");
            if (!target.engaged) Require(target.joints == previous, "disengaged target did not hold");
            for (std::size_t joint = 0; joint < 7; ++joint) {
                Require(std::isfinite(target.joints[joint]), "nonfinite target");
                const float delta = std::abs(target.joints[joint] - previous[joint]);
                Require(delta <= .040001F, "per-sample target delta exceeded 0.04 rad");
                max_delta = std::max(max_delta, delta);
                low[joint] = std::min(low[joint], target.joints[joint]);
                high[joint] = std::max(high[joint], target.joints[joint]);
            }
            if (index) output << ',';
            output << "{\"index\":" << index << ",\"engaged\":" << target.engaged
                   << ",\"calibrated\":" << target.calibrated << ",\"targets\":[";
            for (std::size_t joint = 0; joint < 7; ++joint) {
                if (joint) output << ',';
                output << target.joints[joint];
            }
            output << "]}";
            previous = target.joints;
        }
        for (std::size_t joint = 0; joint < 7; ++joint)
            Require(high[joint] - low[joint] > .005F, "trace did not exercise every arm joint");
        output << "]}\n";
        output.close();
        Require(output.good(), "reference output write failed");
        std::cout << "Validated 1000 production mapper samples; max target delta " << max_delta << " rad\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
