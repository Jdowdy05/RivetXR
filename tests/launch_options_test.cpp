#include "launch_options.h"
#include <cstdio>
#include <stdexcept>

int main() {
    using namespace quest_newton::verification;
    try {
        LaunchOptions options;
        std::string error;
        const auto check = [](bool value) { if (!value) throw std::runtime_error("launch option check failed"); };
        check(ParseLaunchOptions("", "", 1800, options, error));
        check(options.mode == RunMode::live && options.run_id == "live");
        check(ParseLaunchOptions("trace", "run_a-012", 1800, options, error));
        check(options.mode == RunMode::trace && options.run_id == "run_a-012");
        check(ParseLaunchOptions("soak", "runB", 1800, options, error));
        check(options.mode == RunMode::soak && options.soak_seconds == 1800);
        for (const auto mode : {"unknown", "TRACE", " trace"}) {
            check(!ParseLaunchOptions(mode, "run", 1800, options, error));
            check(options.mode == RunMode::soak && options.run_id == "runB");
        }
        for (const auto id : {"", "bad\"id", "../data", "with space"}) {
            check(!ParseLaunchOptions("trace", id, 1800, options, error));
        }
        check(!ParseLaunchOptions("trace", std::string(65, 'x'), 1800, options, error));
        check(!ParseLaunchOptions("soak", "run", 0, options, error));
        check(!ParseLaunchOptions("soak", "run", 3601, options, error));
        check(ParseLaunchOptions("soak", "run", 1, options, error));
        check(ParseLaunchOptions("soak", "run", 3600, options, error));
        check(ParseLaunchOptions("benchmark", "bench_200", 1800, options, error));
        check(options.mode == RunMode::benchmark && options.benchmark_hz == 200 &&
            options.benchmark_seconds == 30 && options.benchmark_warmup_seconds == 8 &&
            options.benchmark_workload == "motion");
        check(ParseLaunchOptions("benchmark", "bounds", 1800, options, error, 50, 5, 0, "rest"));
        check(ParseLaunchOptions("benchmark", "bounds", 1800, options, error, 2000, 600, 60, "motion"));
        for (int hz : {49, 2001}) check(!ParseLaunchOptions("benchmark", "bounds", 1800, options, error, hz));
        for (int seconds : {4, 601}) check(!ParseLaunchOptions("benchmark", "bounds", 1800, options, error, 200, seconds));
        for (int warmup : {-1, 61}) check(!ParseLaunchOptions("benchmark", "bounds", 1800, options, error, 200, 30, warmup));
        check(!ParseLaunchOptions("benchmark", "bounds", 1800, options, error, 200, 30, 8, "random"));
        check(!ParseLaunchOptions("benchmark", "", 1800, options, error));
        std::puts("explicit launch modes and nonce bounds passed");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
