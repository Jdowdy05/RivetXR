#include "quest_newton/franka_kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace quest_newton::kinematics {
namespace {
constexpr float kHalfPi = std::numbers::pi_v<float> * 0.5F;
constexpr double kDamping = 0.05;
constexpr float kCallDelta = 0.04F;
constexpr double kPositionTolerance = 0.0005;
constexpr double kOrientationTolerance = 0.001;
using Residual = std::array<double, 6>;
using Matrix6 = std::array<std::array<double, 6>, 6>;

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
Quaternion Multiply(const Quaternion& a, const Quaternion& b) {
  return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
          a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
          a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
          a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}
Quaternion Conjugate(const Quaternion& q) { return {-q[0], -q[1], -q[2], q[3]}; }
Vec3 Rotate(const Quaternion& q, const Vec3& v) {
  const Vec3 axis{q[0], q[1], q[2]};
  const Vec3 cross = Cross(axis, v);
  const Vec3 twice = Cross(axis, cross);
  return {v[0] + 2.0F * (q[3] * cross[0] + twice[0]),
          v[1] + 2.0F * (q[3] * cross[1] + twice[1]),
          v[2] + 2.0F * (q[3] * cross[2] + twice[2])};
}
Quaternion RotateX(float angle) { return {std::sin(angle * 0.5F), 0, 0, std::cos(angle * 0.5F)}; }
Quaternion RotateZ(float angle) { return {0, 0, std::sin(angle * 0.5F), std::cos(angle * 0.5F)}; }
bool FiniteJoints(const JointVector& joints) {
  return std::all_of(joints.begin(), joints.end(), [](float value) { return std::isfinite(value); });
}

// Direct URDF origins, NOT a DH convention. Each revolute joint's local axis
// is +Z; apply its fixed origin first and then its angle. Source:
// franka_description/robots/panda_arm_hand.urdf (Isaac Sim URDF importer 2.4.31).
constexpr std::array<Vec3, 7> kOrigins{{{0, 0, 0.333F}, {0, 0, 0}, {0, -0.316F, 0},
    {0.0825F, 0, 0}, {-0.0825F, 0.384F, 0}, {0, 0, 0}, {0.088F, 0, 0}}};
constexpr std::array<float, 7> kRolls{0, -kHalfPi, kHalfPi, kHalfPi, -kHalfPi, kHalfPi, kHalfPi};

bool EvaluateChain(const JointVector& joints, Pose& hand, Jacobian* jacobian) {
  if (!FiniteJoints(joints)) return false;
  Pose pose;
  std::array<Vec3, 7> origins{};
  std::array<Vec3, 7> axes{};
  for (std::size_t i = 0; i < 7; ++i) {
    pose = Compose(pose, {kOrigins[i], RotateX(kRolls[i])});
    origins[i] = pose.position;
    axes[i] = Rotate(pose.rotation, {0, 0, 1});
    pose = Compose(pose, {{}, RotateZ(joints[i])});
  }
  pose = Compose(pose, {{0, 0, 0.107F}, RotateZ(-std::numbers::pi_v<float> * 0.25F)});
  if (!NormalizePose(pose, hand)) return false;
  if (jacobian != nullptr) {
    for (std::size_t i = 0; i < 7; ++i) {
      const Vec3 offset{hand.position[0] - origins[i][0], hand.position[1] - origins[i][1], hand.position[2] - origins[i][2]};
      const Vec3 linear = Cross(axes[i], offset);
      for (std::size_t row = 0; row < 3; ++row) {
        (*jacobian)[row][i] = linear[row];
        (*jacobian)[row + 3][i] = axes[i][row];
      }
    }
  }
  return true;
}

Residual PoseResidual(const Pose& current, const Pose& target) {
  Residual result{};
  for (std::size_t i = 0; i < 3; ++i) result[i] = static_cast<double>(target.position[i]) - current.position[i];
  Quaternion error = Multiply(target.rotation, Conjugate(current.rotation));
  // Choose the shortest rotation; canonicalize the exact pi tie as well so
  // target q and -q have the same residual, including a signed-zero scalar.
  bool negate = error[3] < 0;
  if (error[3] == 0) {
    for (std::size_t i = 0; i < 3; ++i) {
      if (error[i] != 0) { negate = error[i] < 0; break; }
    }
  }
  if (negate) for (auto& value : error) value = -value;
  const double sine = std::hypot(static_cast<double>(error[0]), static_cast<double>(error[1]), static_cast<double>(error[2]));
  const double scale = sine > 1e-12 ? 2.0 * std::atan2(sine, static_cast<double>(error[3])) / sine : 2.0;
  for (std::size_t i = 0; i < 3; ++i) result[i + 3] = scale * error[i];
  return result;
}
double SquaredError(const Residual& residual) {
  double result = 0;
  for (const double value : residual) result += value * value;
  return result;
}
bool SetErrors(const Residual& error, IkResult& result) {
  const double position = std::hypot(error[0], error[1], error[2]);
  const double orientation = std::hypot(error[3], error[4], error[5]);
  if (!std::isfinite(position) || !std::isfinite(orientation) || position > std::numeric_limits<float>::max()) return false;
  result.position_error = static_cast<float>(position);
  result.orientation_error = static_cast<float>(orientation);
  result.converged = position <= kPositionTolerance && orientation <= kOrientationTolerance;
  return true;
}

// The damped normal matrix J*J^T + lambda^2*I is positive definite even
// when J is singular. Cholesky avoids inversion and uses fixed stack storage.
bool SolvePositiveDefinite(const Matrix6& matrix, const Residual& rhs, Residual& solution) {
  Matrix6 lower{};
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      double value = matrix[i][j];
      for (std::size_t k = 0; k < j; ++k) value -= lower[i][k] * lower[j][k];
      if (!std::isfinite(value)) return false;
      if (i == j) {
        if (value <= 0) return false;
        lower[i][j] = std::sqrt(value);
      } else {
        lower[i][j] = value / lower[j][j];
      }
    }
  }
  Residual intermediate{};
  for (std::size_t i = 0; i < 6; ++i) {
    double value = rhs[i];
    for (std::size_t j = 0; j < i; ++j) value -= lower[i][j] * intermediate[j];
    intermediate[i] = value / lower[i][i];
  }
  for (std::size_t n = 6; n > 0; --n) {
    const std::size_t i = n - 1;
    double value = intermediate[i];
    for (std::size_t j = i + 1; j < 6; ++j) value -= lower[j][i] * solution[j];
    solution[i] = value / lower[i][i];
    if (!std::isfinite(solution[i])) return false;
  }
  return true;
}
}  // namespace

bool NormalizePose(const Pose& input, Pose& output) {
  for (const float value : input.position) if (!std::isfinite(value)) return false;
  double squared_norm = 0;
  for (const float value : input.rotation) {
    if (!std::isfinite(value)) return false;
    squared_norm += static_cast<double>(value) * value;
  }
  if (squared_norm < 1e-12) return false;
  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  Pose normalized = input;
  for (auto& value : normalized.rotation) value = static_cast<float>(value * inverse_norm);
  output = normalized;
  return true;
}

Pose Compose(const Pose& parent_from_middle, const Pose& middle_from_child) {
  Pose result;
  const Vec3 translation = Rotate(parent_from_middle.rotation, middle_from_child.position);
  for (std::size_t i = 0; i < 3; ++i) result.position[i] = parent_from_middle.position[i] + translation[i];
  result.rotation = Multiply(parent_from_middle.rotation, middle_from_child.rotation);
  // Roundoff from long transform chains must not accumulate scale in rotation.
  double norm = 0;
  for (const float value : result.rotation) norm += static_cast<double>(value) * value;
  const double scale = 1.0 / std::sqrt(norm);
  for (auto& value : result.rotation) value = static_cast<float>(value * scale);
  return result;
}

Pose Inverse(const Pose& pose) {
  Pose result;
  result.rotation = Conjugate(pose.rotation);
  result.position = Rotate(result.rotation, {-pose.position[0], -pose.position[1], -pose.position[2]});
  return result;
}

bool ForwardKinematics(const JointVector& joints, Pose& output) {
  Pose hand;
  if (!EvaluateChain(joints, hand, nullptr)) return false;
  output = hand;
  return true;
}
bool ComputeJacobian(const JointVector& joints, Jacobian& output) {
  Pose hand;
  Jacobian jacobian{};
  if (!EvaluateChain(joints, hand, &jacobian)) return false;
  output = jacobian;
  return true;
}
bool WithinJointLimits(const JointVector& joints) {
  if (!FiniteJoints(joints)) return false;
  for (std::size_t i = 0; i < 7; ++i) {
    if (joints[i] < kLowerLimits[i] || joints[i] > kUpperLimits[i]) return false;
  }
  return true;
}

IkResult SolveIK(const JointVector& seed, const Pose& target) {
  IkResult invalid;
  invalid.joints = seed;
  Pose normalized_target;
  if (!WithinJointLimits(seed) || !NormalizePose(target, normalized_target)) return invalid;
  IkResult result = invalid;
  JointVector lower{};
  JointVector upper{};
  for (std::size_t i = 0; i < 7; ++i) {
    lower[i] = std::max(kLowerLimits[i], seed[i] - kCallDelta);
    upper[i] = std::min(kUpperLimits[i], seed[i] + kCallDelta);
  }
  for (unsigned iteration = 0; iteration < 8; ++iteration) {
    Pose current;
    Jacobian jacobian{};
    if (!EvaluateChain(result.joints, current, &jacobian)) return invalid;
    const Residual error = PoseResidual(current, normalized_target);
    if (!SetErrors(error, result)) return invalid;
    if (result.converged) break;
    Matrix6 normal{};
    for (std::size_t row = 0; row < 6; ++row) {
      for (std::size_t col = 0; col < 6; ++col) {
        for (std::size_t j = 0; j < 7; ++j) normal[row][col] += static_cast<double>(jacobian[row][j]) * jacobian[col][j];
      }
      normal[row][row] += kDamping * kDamping;
    }
    Residual solved{};
    if (!SolvePositiveDefinite(normal, error, solved)) return invalid;
    std::array<double, 7> delta{};
    for (std::size_t j = 0; j < 7; ++j) {
      for (std::size_t row = 0; row < 6; ++row) delta[j] += jacobian[row][j] * solved[row];
      if (!std::isfinite(delta[j])) return invalid;
    }
    ++result.iterations;
    bool accepted = false;
    double scale = 1.0;
    // A short bounded line search prevents projection at joint limits from
    // accepting a step that increases the combined pose residual.
    for (unsigned trial = 0; trial < 5; ++trial, scale *= 0.5) {
      JointVector candidate{};
      for (std::size_t j = 0; j < 7; ++j) {
        const double value = static_cast<double>(result.joints[j]) + scale * delta[j];
        candidate[j] = static_cast<float>(std::clamp(value, static_cast<double>(lower[j]), static_cast<double>(upper[j])));
      }
      Pose candidate_pose;
      if (!ForwardKinematics(candidate, candidate_pose)) return invalid;
      if (SquaredError(PoseResidual(candidate_pose, normalized_target)) <= SquaredError(error)) {
        accepted = true;
        result.joints = candidate;
        break;
      }
    }
    if (!accepted) break;
  }
  Pose final_pose;
  if (!ForwardKinematics(result.joints, final_pose) || !SetErrors(PoseResidual(final_pose, normalized_target), result)) return invalid;
  result.valid = true;
  return result;
}

}  // namespace quest_newton::kinematics
