#include "quest_newton/franka_kinematics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <tuple>

namespace {
using namespace quest_newton::kinematics;
static_assert(std::tuple_size_v<JointVector> == 7);
static_assert(std::tuple_size_v<Jacobian> == 6);

void Check(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(EXIT_FAILURE);
  }
}
bool Near(float actual, float expected, float tolerance = 2e-6F) {
  return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}
float RotationDistance(const Quaternion& a, const Quaternion& b) {
  // Independent chord-distance evaluation is stable at zero and pi.
  float minus = 0.0F;
  float plus = 0.0F;
  for (std::size_t i = 0; i < 4; ++i) {
    minus += (a[i] - b[i]) * (a[i] - b[i]);
    plus += (a[i] + b[i]) * (a[i] + b[i]);
  }
  return 4.0F * std::asin(std::min(1.0F, std::sqrt(std::min(minus, plus)) * 0.5F));
}
void CheckPose(const Pose& actual, const Pose& expected) {
  for (std::size_t i = 0; i < 3; ++i) Check(Near(actual.position[i], expected.position[i]), "golden FK position");
  Check(RotationDistance(actual.rotation, expected.rotation) < 3e-6F, "golden FK orientation");
}
void CheckBounded(const JointVector& seed, const IkResult& result) {
  Check(result.valid, "IK returns valid bounded progress");
  Check(WithinJointLimits(result.joints), "IK hard joint limits");
  Check(result.iterations <= 8, "IK iteration bound");
  Check(std::isfinite(result.position_error) && std::isfinite(result.orientation_error), "finite IK errors");
  for (std::size_t i = 0; i < 7; ++i) {
    Check(std::abs(result.joints[i] - seed[i]) <= 0.040001F, "whole-call joint delta <= .04 rad");
  }
}
void GoldenForwardKinematics() {
  // Independent reference: Python ElementTree parsed the installed Isaac Sim
  // franka_description/robots/panda_arm_hand.urdf on 2026-09-04. NumPy composed
  // 4x4 origin Rz(yaw)*Ry(pitch)*Rx(roll) then joint Rz(q), through joint8 and
  // panda_hand_joint. Quaternions came from a rotation-matrix eigensystem.
  // Zero pose intentionally lies outside joint4's allowed range.
  const JointVector configurations[]{JointVector{}, kHome, {0.4F, -0.8F, 0.2F, -1.9F, -0.3F, 2.1F, 0.5F}};
  const Pose expected[]{
      {{0.088F, 0.0F, 0.926F}, {0.923879532511F, 0.382683432365F, 0.0F, 0.0F}},
      {{0.389447679770F, 0.0F, 0.457823740011F}, {0.921610856850F, 0.020462276078F, 0.387479948194F, 0.008603112280F}},
      {{0.205575079408F, 0.197106629210F, 0.871906556855F}, {0.769678652008F, 0.379338865096F, 0.491142888140F, 0.149918182689F}}};
  for (std::size_t i = 0; i < 3; ++i) {
    Pose actual;
    Check(ForwardKinematics(configurations[i], actual), "finite FK accepted");
    CheckPose(actual, expected[i]);
  }
}
void JacobianFiniteDifference() {
  const JointVector configurations[]{kHome, {0.4F, -0.8F, 0.2F, -1.9F, -0.3F, 2.1F, 0.5F},
      {-1.1F, 0.4F, -0.9F, -0.6F, 1.2F, 0.4F, -0.8F}};
  constexpr float h = 0.001F;
  for (const auto& q : configurations) {
    Jacobian jacobian;
    Pose center;
    Check(ComputeJacobian(q, jacobian) && ForwardKinematics(q, center), "Jacobian finite input");
    for (std::size_t j = 0; j < 7; ++j) {
      JointVector plus = q;
      JointVector minus = q;
      plus[j] += h;
      minus[j] -= h;
      Pose a, b;
      Check(ForwardKinematics(plus, a) && ForwardKinematics(minus, b), "difference FK");
      for (std::size_t row = 0; row < 3; ++row) {
        Check(Near(jacobian[row][j], (a.position[row] - b.position[row]) / (2.0F * h), 4e-4F), "linear Jacobian finite difference");
      }
      // Quaternion derivative: omega = 2 * vector(qdot * conjugate(q)).
      float dot = 0.0F;
      for (std::size_t k = 0; k < 4; ++k) dot += a.rotation[k] * b.rotation[k];
      if (dot < 0.0F) for (auto& value : b.rotation) value = -value;
      Quaternion derivative{};
      for (std::size_t k = 0; k < 4; ++k) derivative[k] = (a.rotation[k] - b.rotation[k]) / (2.0F * h);
      const auto& c = center.rotation;
      const Vec3 omega{
          2.0F * (-derivative[3] * c[0] + derivative[0] * c[3] - derivative[1] * c[2] + derivative[2] * c[1]),
          2.0F * (-derivative[3] * c[1] + derivative[0] * c[2] + derivative[1] * c[3] - derivative[2] * c[0]),
          2.0F * (-derivative[3] * c[2] - derivative[0] * c[1] + derivative[1] * c[0] + derivative[2] * c[3])};
      for (std::size_t row = 0; row < 3; ++row) Check(Near(jacobian[row + 3][j], omega[row], 5e-4F), "angular Jacobian finite difference");
    }
  }
}
void PoseValidationAndComposition() {
  Pose normalized;
  Check(NormalizePose({{1.0F, 2.0F, 3.0F}, {0.0F, 0.0F, 2.0F, 2.0F}}, normalized), "normalize scaled quaternion");
  Check(Near(normalized.rotation[2], std::sqrt(0.5F)), "normalized quaternion magnitude");
  CheckPose(Compose(normalized, Inverse(normalized)), Pose{});
  CheckPose(Compose(Inverse(normalized), normalized), Pose{});
  const Pose child{{1.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F, 1.0F}};
  const Pose composed = Compose(normalized, child);
  Check(Near(composed.position[0], 1.0F) && Near(composed.position[1], 3.0F), "compose rotates translation");
  const float large = std::numeric_limits<float>::max();
  Check(NormalizePose({{}, {large, large, large, large}}, normalized), "normalization avoids float overflow");
  Check(Near(normalized.rotation[0], 0.5F), "large quaternion normalization");
  for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    Pose input;
    input.position[0] = bad;
    Check(!NormalizePose(input, normalized), "nonfinite position rejected");
    input = Pose{};
    input.rotation[0] = bad;
    Check(!NormalizePose(input, normalized), "nonfinite quaternion rejected");
  }
  Check(!NormalizePose({{}, {0, 0, 0, 0}}, normalized), "zero quaternion rejected");
  Check(!NormalizePose({{}, {1e-20F, 0, 0, 0}}, normalized), "degenerate quaternion rejected");
  Check(Near(normalized.rotation[0], 0.5F), "failed normalization preserves output");
}
void InverseKinematics() {
  JointVector target_joints = kHome;
  target_joints[0] += 0.3F;
  target_joints[1] += 0.15F;
  target_joints[3] += 0.2F;
  Pose target;
  Check(ForwardKinematics(target_joints, target), "reachable target FK");
  JointVector q = kHome;
  IkResult result;
  for (unsigned frame = 0; frame < 200; ++frame) {
    result = SolveIK(q, target);
    CheckBounded(q, result);
    q = result.joints;
    if (result.converged) break;
  }
  Check(result.converged, "repeated bounded-frame IK converges to reachable pose");
  Pose reached;
  Check(ForwardKinematics(q, reached), "IK result FK");
  float distance_squared = 0.0F;
  for (std::size_t i = 0; i < 3; ++i) distance_squared += (target.position[i] - reached.position[i]) * (target.position[i] - reached.position[i]);
  Check(std::sqrt(distance_squared) <= 0.001F, "IK reachable position residual");
  Check(RotationDistance(target.rotation, reached.rotation) <= 0.002F, "IK reachable orientation residual");
  Pose same;
  Check(ForwardKinematics(kHome, same), "home FK");
  const auto hold = SolveIK(kHome, same);
  Check(hold.valid && hold.converged && hold.joints == kHome, "home hold makes no change");
  for (auto& value : same.rotation) value = -value;
  const auto opposite = SolveIK(kHome, same);
  Check(opposite.valid && opposite.converged && opposite.joints == kHome, "quaternion double cover holds same pose");
  const JointVector singular{0, 0, 0, -0.0698F, 0, 0, 0};
  Pose singular_pose;
  Check(ForwardKinematics(singular, singular_pose), "near-singular FK");
  const Pose near_pi{{}, {1.0F, 0.0F, 0.0F, 1e-7F}};
  Pose pi_target = Compose(near_pi, singular_pose);
  pi_target.position = singular_pose.position;
  const auto pi_result = SolveIK(singular, pi_target);
  CheckBounded(singular, pi_result);
  Check(pi_result.orientation_error > 3.0F, "near-pi residual does not collapse to zero");
  for (auto& value : pi_target.rotation) value = -value;
  const auto negative_pi_result = SolveIK(singular, pi_target);
  CheckBounded(singular, negative_pi_result);
  for (std::size_t j = 0; j < 7; ++j) Check(Near(pi_result.joints[j], negative_pi_result.joints[j]), "near-pi quaternion sign invariance");
  CheckBounded(kLowerLimits, SolveIK(kLowerLimits, target));
  CheckBounded(kUpperLimits, SolveIK(kUpperLimits, target));
  const auto unreachable = SolveIK(kHome, {{1000, -1000, 1000}, {0, 0, 0, 1}});
  CheckBounded(kHome, unreachable);
  Check(!unreachable.converged, "unreachable target is not converged");
}
void InvalidInputsPreserveOutputs() {
  Pose target;
  Check(ForwardKinematics(kHome, target), "valid target");
  for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    JointVector q = kHome;
    q[0] = bad;
    Pose output = target;
    Jacobian jacobian{};
    jacobian[0][0] = 123;
    Check(!ForwardKinematics(q, output) && output.position == target.position && output.rotation == target.rotation, "bad FK preserves output");
    Check(!ComputeJacobian(q, jacobian) && jacobian[0][0] == 123, "bad Jacobian preserves output");
    Check(!WithinJointLimits(q) && !SolveIK(q, target).valid, "bad joint rejected");
    Pose invalid_target = target;
    invalid_target.position[0] = bad;
    auto result = SolveIK(kHome, invalid_target);
    Check(!result.valid && result.joints == kHome, "invalid IK position preserves seed");
    invalid_target = target;
    invalid_target.rotation[0] = bad;
    result = SolveIK(kHome, invalid_target);
    Check(!result.valid && result.joints == kHome, "invalid IK quaternion preserves seed");
  }
  JointVector outside = kHome;
  outside[3] = 0;
  Check(!SolveIK(outside, target).valid && SolveIK(outside, target).joints == outside, "out-of-limit IK seed rejected and preserved");
  target.rotation = {};
  Check(!SolveIK(kHome, target).valid && SolveIK(kHome, target).joints == kHome, "zero target quaternion rejected");
  const float maximum = std::numeric_limits<float>::max();
  const auto overflow = SolveIK(kHome, {{maximum, maximum, maximum}, {0, 0, 0, 1}});
  Check(!overflow.valid && overflow.joints == kHome, "unrepresentable error preserves seed");
}
}  // namespace

int main() {
  GoldenForwardKinematics();
  JacobianFiniteDifference();
  PoseValidationAndComposition();
  InverseKinematics();
  InvalidInputsPreserveOutputs();
  std::puts("Franka kinematics tests passed");
}
