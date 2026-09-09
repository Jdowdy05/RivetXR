#include "verification_state_limits.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>

int main() {
    using namespace quest_newton;
    using namespace quest_newton::verification;
    const auto check = [](bool v) { if (!v) throw std::runtime_error("state allowance check failed"); };
    try {
        const std::array<float,9> home{0,-.569F,0,-2.81F,0,3.037F,.741F,.04F,.04F};
        StateLimitStats stats;
        check(stats.Observe(home));
        auto q = home;
        q[7] = .0400005F;
        const auto unchanged = q;
        check(stats.Observe(q));
        check(q == unchanged);
        check(stats.samples_checked == 2 && stats.strict_excursion_samples == 1);
        check(stats.per_joint_excursion_counts[7] == 1);
        check(stats.max_excursions[7] > 4e-7 && stats.max_excursions[7] < 6e-7);
        q = home;
        q[0] = kinematics::kUpperLimits[0] + .000005F;
        check(stats.Observe(q));
        auto target = kinematics::kHome;
        target[0] = q[0];
        check(!kinematics::WithinJointLimits(target)); // State allowance never extends target bounds.
        q = home;
        q[7] = .040002F;
        check(!stats.Observe(q));
        check(stats.allowance_violations == 1 && stats.max_excursions[7] > 1e-6);
        q = home;
        q[0] = kinematics::kUpperLimits[0] + .00002F;
        check(!stats.Observe(q));
        check(stats.allowance_violations == 2);
        q = home;
        q[8] = -.0000005F;
        check(stats.Observe(q));
        check(stats.per_joint_excursion_counts[8] == 1);
        const auto checked = stats.samples_checked;
        q[8] = std::numeric_limits<float>::infinity();
        check(!stats.Observe(q) && stats.samples_checked == checked);
        check(!stats.Observe(std::span(home).first(8)) && stats.samples_checked == checked);
        std::ostringstream policy, report;
        WriteStateLimitPolicy(policy);
        stats.WriteJson(report);
        check(policy.str().find("newton_soft_stop_state_v1") != std::string::npos);
        check(report.str().find("\"allowance_violations\":2") != std::string::npos);
        std::puts("approved state allowances, raw excursion counts, and strict target bounds passed");
    } catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }
}
