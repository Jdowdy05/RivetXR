#pragma once

#include <array>

namespace quest_newton::kinematics {

using Vec3 = std::array<float, 3>;
// Quaternion component order is XYZW.
using Quaternion = std::array<float, 4>;
using JointVector = std::array<float, 7>;
// Base-frame linear XYZ rows followed by base-frame angular XYZ rows.
using Jacobian = std::array<std::array<float, 7>, 6>;

struct Pose {
  Vec3 position{};
  Quaternion rotation{0.0F, 0.0F, 0.0F, 1.0F};
};

inline constexpr JointVector kHome{0.0F, -0.569F, 0.0F, -2.810F, 0.0F, 3.037F, 0.741F};
inline constexpr JointVector kLowerLimits{-2.8973F, -1.7628F, -2.8973F, -3.0718F,
                                         -2.8973F, -0.0175F, -2.8973F};
inline constexpr JointVector kUpperLimits{2.8973F, 1.7628F, 2.8973F, -0.0698F,
                                         2.8973F, 3.7525F, 2.8973F};

// Rejects nonfinite positions/quaternions and degenerate quaternions, without
// changing output. Otherwise normalizes the quaternion, including scaled input.
bool NormalizePose(const Pose& input, Pose& output);
// These transform helpers require finite positions and unit quaternions.
Pose Compose(const Pose& parent_from_middle, const Pose& middle_from_child);
Pose Inverse(const Pose& parent_from_child);

// Base -> panda_hand palm origin, including joint8 and the fixed hand rotation.
// Finite out-of-limit joints are accepted for reference FK/Jacobian evaluation.
// Failure leaves output unchanged. Distances are metres and angles are radians.
bool ForwardKinematics(const JointVector& joints, Pose& output);
bool ComputeJacobian(const JointVector& joints, Jacobian& output);
bool WithinJointLimits(const JointVector& joints);

struct IkResult {
  JointVector joints{};
  bool valid = false;
  bool converged = false;
  float position_error = 0.0F;
  float orientation_error = 0.0F;
  unsigned iterations = 0;
};

// At most eight damped least-squares iterations (damping 0.05). Every output
// joint stays within URDF limits and 0.04 rad of its input seed for the WHOLE
// call. Valid means finite bounded output; it does not imply convergence.
// Invalid input or numerical failure preserves seed and returns valid=false.
IkResult SolveIK(const JointVector& seed, const Pose& target);

}  // namespace quest_newton::kinematics
