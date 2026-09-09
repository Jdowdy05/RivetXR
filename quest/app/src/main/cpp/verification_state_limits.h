#pragma once
#include "quest_newton/franka_kinematics.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>
#include <span>

namespace quest_newton::verification {
inline constexpr double kArmStateAllowance = 1e-5;
inline constexpr double kFingerStateAllowance = 1e-6;
inline constexpr auto kStateLower = [] {
    std::array<float,9> values{};
    for (std::size_t i=0; i<7; ++i) values[i] = kinematics::kLowerLimits[i];
    return values;
}();
inline constexpr auto kStateUpper = [] {
    std::array<float,9> values{};
    for (std::size_t i=0; i<7; ++i) values[i] = kinematics::kUpperLimits[i];
    values[7] = values[8] = .04F;
    return values;
}();
inline void WriteStateLimitPolicy(std::ostream& out) {
    out << "{\"name\":\"newton_soft_stop_state_v1\",\"arm_rad\":1e-5,\"finger_m\":1e-6,\"target_limits\":\"strict\"}";
}

// Verification policy only. Never modifies simulation state or target limits.
struct StateLimitStats {
    std::uint64_t samples_checked = 0;
    std::uint64_t strict_excursion_samples = 0;
    std::array<std::uint64_t,9> per_joint_excursion_counts{};
    std::array<double,9> max_excursions{};
    std::uint64_t allowance_violations = 0;

    bool Observe(std::span<const float> q) {
        if (q.size() != 9 || samples_checked == std::numeric_limits<std::uint64_t>::max()) return false;
        std::array<double,9> excursions{};
        bool any = false, violation = false;
        for (std::size_t i=0; i<9; ++i) {
            if (!std::isfinite(q[i])) return false;
            const double value = q[i];
            excursions[i] = std::max({0.0, static_cast<double>(kStateLower[i]) - value,
                                    value - static_cast<double>(kStateUpper[i])});
            any |= excursions[i] > 0;
            violation |= excursions[i] > (i < 7 ? kArmStateAllowance : kFingerStateAllowance);
        }
        ++samples_checked;
        if (any) ++strict_excursion_samples;
        if (violation) ++allowance_violations;
        for (std::size_t i=0; i<9; ++i) {
            if (excursions[i] > 0) ++per_joint_excursion_counts[i];
            max_excursions[i] = std::max(max_excursions[i], excursions[i]);
        }
        return !violation;
    }
    void WriteJson(std::ostream& out) const {
        out << "{\"samples_checked\":" << samples_checked << ",\"strict_excursion_samples\":" << strict_excursion_samples
            << ",\"per_joint_excursion_counts\":[";
        for (std::size_t i=0; i<9; ++i) { if (i) out << ','; out << per_joint_excursion_counts[i]; }
        out << "],\"max_excursions\":[";
        for (std::size_t i=0; i<9; ++i) { if (i) out << ','; out << max_excursions[i]; }
        out << "],\"allowance_violations\":" << allowance_violations << '}';
    }
};
} // namespace quest_newton::verification
