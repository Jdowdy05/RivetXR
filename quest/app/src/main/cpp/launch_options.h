#pragma once

#include <string>
#include <string_view>

namespace quest_newton::verification {
enum class RunMode { live, trace, soak, benchmark };
struct LaunchOptions {
    RunMode mode = RunMode::live;
    std::string run_id = "live";
    int soak_seconds = 1800;
    int benchmark_hz = 200;
    int benchmark_seconds = 30;
    int benchmark_warmup_seconds = 8;
    std::string benchmark_workload = "motion";
};
inline const char* ModeName(RunMode mode) {
    return mode == RunMode::benchmark ? "benchmark" : mode == RunMode::trace ? "trace" : mode == RunMode::soak ? "soak" : "live";
}
inline bool ParseLaunchOptions(std::string_view mode, std::string_view run_id, int soak_seconds,
                              LaunchOptions& output, std::string& error,
                              int benchmark_hz = 200, int benchmark_seconds = 30,
                              int benchmark_warmup_seconds = 8, std::string_view benchmark_workload = "motion") {
    LaunchOptions candidate;
    if (mode.empty() || mode == "live") candidate.mode = RunMode::live;
    else if (mode == "trace") candidate.mode = RunMode::trace;
    else if (mode == "soak") candidate.mode = RunMode::soak;
    else if (mode == "benchmark") candidate.mode = RunMode::benchmark;
    else { error = "unknown launch mode"; return false; }
    if (run_id.empty() && candidate.mode == RunMode::live) run_id = "live";
    if (run_id.empty() || run_id.size() > 64) { error = "verification run id must have 1-64 characters"; return false; }
    for (const char c : run_id) {
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-')) {
            error = "invalid verification run id";
            return false;
        }
    }
    if (soak_seconds < 1 || soak_seconds > 3600) { error = "soak duration must be 1-3600 seconds"; return false; }
    candidate.run_id = run_id;
    candidate.soak_seconds = soak_seconds;
    if (benchmark_hz < 50 || benchmark_hz > 2000) { error = "benchmark Hz must be 50-2000"; return false; }
    if (benchmark_seconds < 5 || benchmark_seconds > 600) { error = "benchmark duration must be 5-600 seconds"; return false; }
    if (benchmark_warmup_seconds < 0 || benchmark_warmup_seconds > 60) { error = "benchmark warmup must be 0-60 seconds"; return false; }
    if (benchmark_workload != "motion" && benchmark_workload != "rest") { error = "benchmark workload must be motion or rest"; return false; }
    candidate.benchmark_hz = benchmark_hz;
    candidate.benchmark_seconds = benchmark_seconds;
    candidate.benchmark_warmup_seconds = benchmark_warmup_seconds;
    candidate.benchmark_workload = benchmark_workload;
    output = candidate;
    error.clear();
    return true;
}
} // namespace quest_newton::verification
