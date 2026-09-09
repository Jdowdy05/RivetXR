#include "input_mapper.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace {
namespace kin = quest_newton::kinematics;
using quest_newton::ControllerSample;
using quest_newton::InputMapper;
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void Near(float actual, float expected, float tolerance = 2e-5F) {
    Check(std::isfinite(actual) && std::abs(actual - expected) < tolerance, "numeric mismatch");
}
ControllerSample Tracked() {
    ControllerSample sample;
    // Independently evaluated URDF home panda_hand orientation (XYZW).
    sample.stage_from_grip = {{1, 2, 3},
        {.921610856850F, .020462276078F, .387479948194F, .008603112280F}};
    sample.focused = sample.stage_valid = sample.pose_active = true;
    sample.trigger_active = sample.calibrate_active = true;
    sample.position_valid = sample.orientation_valid = true;
    sample.position_tracked = sample.orientation_tracked = true;
    return sample;
}
void Calibrate(InputMapper& mapper, ControllerSample& sample) {
    sample.trigger = 0;
    sample.calibrate_pressed = false;
    mapper.Update(sample);
    sample.calibrate_pressed = true;
    Check(mapper.Update(sample).calibrated, "explicit released calibration failed");
    sample.calibrate_pressed = false;
    mapper.Update(sample);
}
void Engage(InputMapper& mapper, ControllerSample& sample) {
    Calibrate(mapper, sample);
    sample.trigger = 1;
    Check(mapper.Update(sample).engaged, "released then pressed trigger must engage");
}

void NoAutomaticCalibrationOrHeldTriggerCalibration() {
    InputMapper mapper;
    auto sample = Tracked();
    mapper.Update(sample);
    sample.trigger = 1;
    Check(!mapper.Update(sample).engaged, "uncalibrated trigger engaged");
    sample.calibrate_pressed = true;
    Check(!mapper.Update(sample).calibrated, "calibration while trigger held");
    sample.trigger = 0;
    Check(!mapper.Update(sample).calibrated, "held calibration press retriggered");
    Calibrate(mapper, sample);
}

void ReleasedTriggerFreezesAndUsesHysteresis() {
    InputMapper mapper;
    auto sample = Tracked();
    Calibrate(mapper, sample);
    sample.trigger = .5F;
    Check(!mapper.Update(sample).engaged, "engaged at strict high boundary");
    sample.trigger = .51F;
    Check(mapper.Update(sample).engaged, "high trigger did not engage");
    sample.trigger = .2F;
    Check(mapper.Update(sample).engaged, "hysteresis low boundary disengaged");
    sample.trigger = .19F;
    sample.calibrate_active = false;
    const auto held = mapper.Current().joints;
    sample.stage_from_grip.position[0] += .2F;
    Check(!mapper.Update(sample).engaged, "release with inactive calibration failed");
    Check(mapper.Current().joints == held, "released trigger changed targets");
    sample.trigger = .4F;
    Check(!mapper.Update(sample).engaged, "midband reengaged");
    sample.trigger = 1;
    Check(mapper.Update(sample).engaged, "calibration action incorrectly gates control");
}

void TrackingFailuresRequireReleaseAndPreserveTargets() {
    bool ControllerSample::* flags[] = {
        &ControllerSample::focused, &ControllerSample::stage_valid,
        &ControllerSample::pose_active, &ControllerSample::trigger_active,
        &ControllerSample::position_valid, &ControllerSample::orientation_valid,
        &ControllerSample::position_tracked, &ControllerSample::orientation_tracked};
    for (auto flag : flags) {
        InputMapper mapper;
        auto sample = Tracked();
        Engage(mapper, sample);
        const auto held = mapper.Current().joints;
        auto invalid = sample;
        // These members are intentionally mutated independently.
        invalid.*flag = false;
        Check(!mapper.Update(invalid).engaged, "missing tracking/focus/action flag accepted");
        Check(mapper.Current().joints == held, "tracking failure changed targets");
        Check(!mapper.Update(sample).engaged, "held trigger reacquired after tracking failure");
        sample.trigger = .2F;
        mapper.Update(sample);
        sample.trigger = 1;
        Check(!mapper.Update(sample).engaged, "low boundary counted as release");
        sample.trigger = .19F;
        mapper.Update(sample);
        sample.trigger = 1;
        Check(mapper.Update(sample).engaged, "release did not rearm after loss");

        InputMapper uncalibrated;
        invalid.trigger = 0;
        invalid.calibrate_pressed = true;
        Check(!uncalibrated.Update(invalid).calibrated, "invalid tracking allowed calibration");
    }
}

void InvalidNumbersFailClosed() {
    const float bad_values[] = {std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
    for (float bad : bad_values) {
        for (std::size_t element = 0; element < 8; ++element) {
            InputMapper mapper;
            auto sample = Tracked();
            Engage(mapper, sample);
            const auto held = mapper.Current().joints;
            auto invalid = sample;
            if (element < 3) invalid.stage_from_grip.position[element] = bad;
            else if (element < 7) invalid.stage_from_grip.rotation[element - 3] = bad;
            else invalid.trigger = bad;
            Check(!mapper.Update(invalid).engaged, "nonfinite sample accepted");
            Check(mapper.Current().joints == held, "nonfinite sample changed targets");
            Check(!mapper.Update(sample).engaged, "nonfinite failure did not require release");
        }
    }
    for (float trigger : {-.01F, 1.01F}) {
        InputMapper mapper;
        auto sample = Tracked();
        Engage(mapper, sample);
        sample.trigger = trigger;
        Check(!mapper.Update(sample).engaged, "out of range trigger accepted");
        sample.trigger = 1;
        Check(!mapper.Update(sample).engaged, "invalid trigger counted as release");
    }
    InputMapper mapper;
    auto sample = Tracked();
    Engage(mapper, sample);
    sample.stage_from_grip.rotation = {0, 0, 0, 0};
    Check(!mapper.Update(sample).engaged, "zero quaternion accepted");
    sample = Tracked();
    Calibrate(mapper, sample);
    sample.trigger = 1;
    sample.stage_from_grip.rotation = {1e-12F, 1e-12F, 1e-12F, 1e-12F};
    Check(!mapper.Update(sample).engaged, "degenerate quaternion accepted");
}

void CalibrationEdgesAndInvalidation() {
    InputMapper mapper;
    auto sample = Tracked();
    sample.calibrate_pressed = true;
    sample.focused = false;
    mapper.Update(sample);
    sample.focused = true;
    Check(!mapper.Update(sample).calibrated, "held invalid press reactivated on focus");
    Calibrate(mapper, sample);
    sample.calibrate_pressed = true;
    mapper.Update(sample);
    const auto base = mapper.Current().robot_base_from_stage;
    sample.stage_from_grip.position[0] += 1;
    mapper.Update(sample);
    Check(mapper.Current().robot_base_from_stage.position == base.position,
          "held calibration moves the base repeatedly");
    const auto held = mapper.Current().joints;
    mapper.InvalidateCalibration();
    Check(!mapper.Current().calibrated && !mapper.Current().engaged,
          "stage invalidation retained calibration");
    Check(mapper.Current().joints == held, "stage invalidation changed held targets");
    Check(!mapper.Update(sample).calibrated, "invalidation turned held button into press");
    sample.trigger = 1;
    Check(!mapper.Update(sample).engaged, "stage invalidation did not disarm");
    Calibrate(mapper, sample);

    // An adapter may return an entirely default sample on an API failure. Its
    // false button bit must not count as an observed physical button release.
    sample.calibrate_pressed = true;
    mapper.Update(sample);
    const auto before_failure = mapper.Current().robot_base_from_stage;
    mapper.Update(ControllerSample{});
    sample.stage_from_grip.position[0] += 1;
    mapper.Update(sample);
    Check(mapper.Current().robot_base_from_stage.position == before_failure.position,
          "default failed sample turned a held button into calibration press");
}

void MissingCalibrationActionCannotCalibrate() {
    InputMapper mapper;
    auto sample = Tracked();
    sample.calibrate_active = false;
    sample.calibrate_pressed = true;
    Check(!mapper.Update(sample).calibrated, "inactive calibration action calibrated");
    sample.calibrate_active = true;
    Check(!mapper.Update(sample).calibrated, "held inactive button reactivated");
    Calibrate(mapper, sample);
    sample.calibrate_active = false;
    sample.trigger = 1;
    Check(mapper.Update(sample).engaged, "inactive calibration action blocks valid control");
    sample.stage_from_grip.position[0] += .02F;
    Check(mapper.Update(sample).engaged, "inactive calibration action interrupted control");
}

void NumericalIkFailureRequiresRelease() {
    InputMapper mapper;
    auto sample = Tracked();
    Engage(mapper, sample);
    const auto held = mapper.Current().joints;
    const float huge = std::numeric_limits<float>::max() * .75F;
    sample.stage_from_grip.position = {huge, huge, huge};
    // Each component is representable, but the position residual norm is not.
    Check(!mapper.Update(sample).engaged, "numerically invalid IK stayed engaged");
    Check(mapper.Current().reason == "invalid inverse kinematics", "did not exercise IK failure");
    Check(mapper.Current().joints == held, "IK failure overwrote finite target");
    sample.stage_from_grip = Tracked().stage_from_grip;
    Check(!mapper.Update(sample).engaged, "IK failure reacquired with held trigger");
    sample.trigger = 0;
    mapper.Update(sample);
    sample.trigger = 1;
    Check(mapper.Update(sample).engaged, "release did not rearm after IK failure");
}

void RecalibrationUsesHeldCommandedJoints() {
    InputMapper mapper;
    auto sample = Tracked();
    Engage(mapper, sample);
    sample.stage_from_grip.position[1] += .1F;
    for (int frame = 0; frame < 20; ++frame) mapper.Update(sample);
    const auto held = mapper.Current().joints;
    Check(held != kin::kHome, "controller movement did not produce a new command");
    kin::Pose hand_at_held_target;
    Check(kin::ForwardKinematics(held, hand_at_held_target), "finite held target has no FK");
    sample.stage_from_grip.position = {3, 4, 5};
    sample.stage_from_grip.rotation = {0, 0, 0, 1};
    Calibrate(mapper, sample);
    Check(mapper.Current().joints == held, "recalibration reset joints to home");
    for (std::size_t j = 0; j < 3; ++j) {
        Near(mapper.Current().robot_base_from_grip.position[j], hand_at_held_target.position[j]);
    }
    for (std::size_t j = 0; j < 4; ++j) {
        Near(mapper.Current().robot_base_from_grip.rotation[j], hand_at_held_target.rotation[j]);
    }
    sample.trigger = 1;
    mapper.Update(sample);
    for (std::size_t j = 0; j < 7; ++j) Near(mapper.Current().joints[j], held[j]);
}

void AbsoluteMappingAndBoundedFarTargets() {
    InputMapper mapper;
    auto sample = Tracked();
    for (float& component : sample.stage_from_grip.rotation) component *= 2;
    Calibrate(mapper, sample);
    const auto initial = mapper.Current();
    // Matching grip/hand rotations cancel. Translation is home minus (1,2,3).
    Near(initial.robot_base_from_stage.position[0], -.610552320230F);
    Near(initial.robot_base_from_stage.position[1], -2);
    Near(initial.robot_base_from_stage.position[2], -2.542176259989F);
    Near(initial.robot_base_from_stage.rotation[0], 0);
    Near(initial.robot_base_from_stage.rotation[1], 0);
    Near(initial.robot_base_from_stage.rotation[2], 0);
    Near(std::abs(initial.robot_base_from_stage.rotation[3]), 1);
    Near(initial.robot_base_from_grip.position[0], .389447679770F);
    Near(initial.robot_base_from_grip.position[1], 0);
    Near(initial.robot_base_from_grip.position[2], .457823740011F);
    sample.trigger = 1;
    mapper.Update(sample);
    for (std::size_t j = 0; j < 7; ++j) Near(mapper.Current().joints[j], kin::kHome[j]);
    for (std::size_t j = 0; j < 4; ++j) {
        Near(mapper.Current().robot_base_from_grip.rotation[j],
             Tracked().stage_from_grip.rotation[j]);
    }
    // No view/head pose appears in this API. Repeated STAGE grip samples keep
    // the same absolute target even as the caller renders a different view.
    for (int frame = 0; frame < 5; ++frame) {
        mapper.Update(sample);
        for (std::size_t j = 0; j < 7; ++j) Near(mapper.Current().joints[j], kin::kHome[j]);
    }
    sample.stage_from_grip.position[0] += .1F;
    mapper.Update(sample);
    Near(mapper.Current().robot_base_from_grip.position[0],
         initial.robot_base_from_grip.position[0] + .1F);
    Near(mapper.Current().robot_base_from_grip.position[1], initial.robot_base_from_grip.position[1]);
    Near(mapper.Current().robot_base_from_grip.position[2], initial.robot_base_from_grip.position[2]);
    sample.stage_from_grip.rotation = {0, 0, 0, 1};
    mapper.Update(sample);
    Near(mapper.Current().robot_base_from_grip.rotation[0], 0);
    Near(mapper.Current().robot_base_from_grip.rotation[1], 0);
    Near(mapper.Current().robot_base_from_grip.rotation[2], 0);
    Near(std::abs(mapper.Current().robot_base_from_grip.rotation[3]), 1);
    const auto fixed_base = mapper.Current().robot_base_from_stage;
    sample.stage_from_grip.position = {100, 100, 100};
    for (int frame = 0; frame < 10; ++frame) {
        const auto previous = mapper.Current().joints;
        Check(mapper.Update(sample).engaged, "finite unreachable target must stay bounded");
        for (std::size_t j = 0; j < 7; ++j) {
            Check(std::isfinite(mapper.Current().joints[j]), "nonfinite IK target escaped");
            Check(std::abs(mapper.Current().joints[j] - previous[j]) <= .040001F,
                  "per-frame IK delta exceeded");
        }
        Check(mapper.Current().robot_base_from_stage.position == fixed_base.position &&
              mapper.Current().robot_base_from_stage.rotation == fixed_base.rotation,
              "control drifted stage-fixed calibration");
    }
}
}  // namespace

int main() {
    try {
        NoAutomaticCalibrationOrHeldTriggerCalibration();
        ReleasedTriggerFreezesAndUsesHysteresis();
        TrackingFailuresRequireReleaseAndPreserveTargets();
        InvalidNumbersFailClosed();
        CalibrationEdgesAndInvalidation();
        MissingCalibrationActionCannotCalibrate();
        NumericalIkFailureRequiresRelease();
        RecalibrationUsesHeldCommandedJoints();
        AbsoluteMappingAndBoundedFarTargets();
        std::puts("input mapper tests passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "input mapper test failed: %s\n", error.what());
        return 1;
    }
}
