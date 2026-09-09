#include "quest_newton_app.h"
#include "controller_trace.h"
#include "controller_trace_data.h"
#include "quest_newton/newton_runtime.h"

#include <android/log.h>
#include <android/trace.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <span>
#include <sstream>
#include <stdexcept>

namespace quest_newton {
namespace {
using Clock = std::chrono::steady_clock;

template<class Range>
void WriteArray(std::ostream& out, const Range& values) {
    out << '[';
    bool separator = false;
    for (const auto value : values) {
        if (!std::isfinite(value)) throw std::runtime_error("non-finite verification output");
        if (separator) out << ',';
        out << value;
        separator = true;
    }
    out << ']';
}

void WriteTiming(std::ostream& out, const verification::TimingSummary& value) {
    out << "{\"available\":" << value.available << ",\"count\":" << value.count
        << ",\"mean_us\":" << value.mean_us << ",\"p95_us\":" << value.p95_us
        << ",\"p99_us\":" << value.p99_us << ",\"max_us\":" << value.max_us
        << ",\"over_budget_count\":" << value.over_budget_count << '}';
}

void OpenOutput(std::ofstream& stream, const std::filesystem::path& path) {
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream.imbue(std::locale::classic());
    stream.open(path, std::ios::out | std::ios::trunc);
    stream << std::boolalpha << std::setprecision(17);
}

bool ValidState(const NewtonRuntime& simulation, verification::StateLimitStats& limits, std::string& detail) {
    const auto q = simulation.JointPositions();
    const auto qd = simulation.JointVelocities();
    const auto bodies = simulation.BodyTransforms();
    if (q.size() != 9 || qd.size() != 9 || bodies.size() != 84) {
        detail = "incorrect verification state dimensions";
        return false;
    }
    for (std::size_t i = 0; i < q.size(); ++i) {
        if (!std::isfinite(q[i]) || !std::isfinite(qd[i])) {
            detail = "non-finite verification joint state";
            return false;
        }
    }
    std::array<Pose, kArmBodies> decoded;
    if (!DecodeBodies(bodies, decoded)) {
        detail = "invalid complete verification body transforms";
        return false;
    }
    if (!limits.Observe(q)) {
        detail = "state excursion exceeds approved soft-stop verification allowance";
        return false;
    }
    return true;
}
} // namespace

bool QuestNewtonApp::ValidateVerificationState(const NewtonRuntime& simulation, std::string& detail) {
    return ValidState(simulation, verification_state_limits_, detail);
}

void QuestNewtonApp::RunVerification(NewtonRuntime& simulation) {
    using namespace std::chrono_literals;
    static_assert(generated_trace::kSamples.size() == 1000);
    static_assert(generated_trace::kSubstepsPerSample == 10);
    const bool trace = launch_options_.mode == verification::RunMode::trace;
    const auto root = std::filesystem::path(internal_data_path_);
    const auto trace_final = root / "newton_trace.jsonl";
    const auto trace_pending = root / "newton_trace.pending";
    const auto metrics_final = root / "newton_metrics.json";
    const auto metrics_pending = root / "newton_metrics.pending";
    // A failed/aborted new run must never leave an old complete result visible.
    std::filesystem::remove(trace_final);
    std::filesystem::remove(metrics_final);
    {
        std::unique_lock lock(wait_mutex_);
        const bool ready = wake_worker_.wait_for(lock, 30s, [this] {
            return stop_worker_.load() || verification_frame_available_.load();
        });
        if (stop_worker_.load()) return;
        if (!ready) {
            Fail("verification_ready", "no focused 90 Hz render frame within 30 seconds");
            return;
        }
    }
    verification_interrupted_.store(false);
    verification_running_.store(true);
    struct RunningGuard {
        std::atomic<bool>& running;
        ~RunningGuard() { running.store(false); }
    } running_guard{verification_running_};

    std::ofstream records, progress;
    if (trace) {
        OpenOutput(records, trace_pending);
        records << "{\"type\":\"header\",\"schema_version\":1,\"mode\":\"trace\",\"run_id\":\""
            << launch_options_.run_id << "\",\"trace_sha256\":\"" << generated_trace::kTraceSha256
            << "\",\"physics_manifest_sha256\":\"" << generated_trace::kPhysicsManifestSha256
            << "\",\"state_limit_policy\":";
        verification::WriteStateLimitPolicy(records);
        records << ",\"substeps_per_sample\":10,\"state_count\":1000}\n";
    } else {
        OpenOutput(progress, root / "newton_soak_progress.jsonl");
    }

    constexpr std::array<float, 9> home{0, -.569F, 0, -2.810F, 0, 3.037F, .741F, .04F, .04F};
    constexpr std::array<float, 9> zero{};
    InputMapper mapper;
    std::uint64_t state_count = 0;
    std::size_t sample_index = 0;
    bool smoke_reported = false;
    const auto begin = Clock::now();
    auto next_progress = begin + 10s;
    while (!stop_worker_.load()) {
        const auto cycle_start = Clock::now();
        if (verification_interrupted_.load() || !verification_frame_available_.load() ||
            std::abs(refresh_hz_.load() - 90.0F) >= .1F) {
            Fail("verification_interrupted", "focus, STAGE, or refresh rate changed during verification");
            return;
        }
        ATrace_beginSection("QuestNewton.RecordedIK");
        const auto mapped = mapper.Update(verification::ToControllerSample(generated_trace::kSamples[sample_index]));
        ATrace_endSection();
        if (mapped.engaged) {
            auto target = home;
            std::copy(mapped.joints.begin(), mapped.joints.end(), target.begin());
            const auto status = simulation.SetJointTargets(target, zero);
            if (!status.IsOk()) { Fail("verification_targets", status.message); return; }
        }
        for (int substep = 0; substep < 10; ++substep) {
            if (!TimedStep(simulation, true)) {
                __android_log_print(ANDROID_LOG_ERROR, "QuestNewton", "QUEST_NEWTON_VERIFY_CONTEXT sample=%zu substep=%d",
                                    sample_index, substep);
                return;
            }
        }
        Mat4 placement = ProvisionalStagePlacement();
        if (mapped.calibrated) {
            const auto pose = kinematics::Inverse(mapped.robot_base_from_stage);
            placement = PoseMatrix({pose.position, pose.rotation});
        }
        if (!mailbox_.Publish(simulation.BodyTransforms(), &placement)) {
            Fail("verification_publish", "complete recorded snapshot rejected");
            return;
        }
        if (!smoke_reported) {
            __android_log_print(ANDROID_LOG_INFO, "QuestNewton", "QUEST_NEWTON_SMOKE_OK first=0.000000 last=%.9g",
                                static_cast<double>(simulation.JointPositions()[0]));
            smoke_reported = true;
        }
        ++state_count;
        if (trace) {
            records << "{\"type\":\"state\",\"index\":" << sample_index
                << ",\"sim_time_s\":" << static_cast<double>(sample_index + 1) * .01
                << ",\"engaged\":" << mapped.engaged << ",\"calibrated\":" << mapped.calibrated
                << ",\"targets\":";
            WriteArray(records, mapped.joints);
            records << ",\"q\":"; WriteArray(records, simulation.JointPositions());
            records << ",\"qd\":"; WriteArray(records, simulation.JointVelocities());
            records << ",\"body_q\":"; WriteArray(records, simulation.BodyTransforms());
            records << "}\n";
        }
        const auto now = Clock::now();
        if (!trace && now >= next_progress) {
            const double elapsed = std::chrono::duration<double>(now - begin).count();
            progress << "{\"run_id\":\"" << launch_options_.run_id << "\",\"elapsed_seconds\":" << elapsed
                << ",\"valid_state_count\":" << state_count << ",\"physics_count\":" << state_count * 10
                << ",\"render_frames\":" << render_frames_.load() << ",\"refresh_hz\":" << refresh_hz_.load() << "}\n";
            progress.flush();
            __android_log_print(ANDROID_LOG_INFO, "QuestNewton", "QUEST_NEWTON_SOAK_PROGRESS run_id=%s elapsed=%.1f states=%llu",
                launch_options_.run_id.c_str(), elapsed, static_cast<unsigned long long>(state_count));
            next_progress = now + 10s;
        }
        ++sample_index;
        if (trace && sample_index == generated_trace::kSamples.size()) break;
        if (!trace && std::chrono::duration<double>(now - begin).count() >= launch_options_.soak_seconds) break;
        if (sample_index == generated_trace::kSamples.size()) {
            const auto status = simulation.Reset(home, zero);
            if (!status.IsOk()) { Fail("verification_reset", status.message); return; }
            mapper = InputMapper{};
            sample_index = 0;
        }
        const auto wake_at = std::max(cycle_start + 10ms, Clock::now() + 1ms);
        std::unique_lock lock(wait_mutex_);
        wake_worker_.wait_until(lock, wake_at, [this] { return stop_worker_.load(); });
    }
    if (stop_worker_.load() || verification_interrupted_.load()) return;
    const double duration = std::chrono::duration<double>(Clock::now() - begin).count();
    const auto physics = physics_timing_->Snapshot();
    // GPU samples refer to older completed frames. Capture them before CPU
    // count so concurrent rendering cannot make GPU coverage exceed the frame
    // snapshot. Derive render_frames from this same CPU snapshot.
    const auto render_gpu = render_gpu_timing_->Snapshot();
    const auto render_cpu = render_cpu_timing_->Snapshot();
    if (trace) {
        records << "{\"type\":\"complete\",\"state_count\":" << state_count << ",\"physics\":";
        WriteTiming(records, physics);
        records << ",\"state_limits\":";
        verification_state_limits_.WriteJson(records);
        records << "}\n";
        records.flush();
        records.close();
        std::filesystem::rename(trace_pending, trace_final);
    } else {
        progress.flush();
        progress.close();
    }
    std::ofstream metrics;
    OpenOutput(metrics, metrics_pending);
    metrics << "{\"schema_version\":1,\"mode\":\"" << verification::ModeName(launch_options_.mode)
        << "\",\"run_id\":\"" << launch_options_.run_id << "\",\"trace_sha256\":\"" << generated_trace::kTraceSha256
        << "\",\"physics_manifest_sha256\":\"" << generated_trace::kPhysicsManifestSha256
        << "\",\"state_limit_policy\":";
    verification::WriteStateLimitPolicy(metrics);
    metrics << ",\"state_limits\":";
    verification_state_limits_.WriteJson(metrics);
    metrics << ",\"completed\":true,\"duration_seconds\":" << duration
        << ",\"soak_seconds_requested\":" << (trace ? 0 : launch_options_.soak_seconds)
        << ",\"valid_state_count\":" << state_count << ",\"render_frames\":" << render_cpu.count
        << ",\"refresh_hz\":" << refresh_hz_.load() << ",\"gpu_timer_supported\":" << gpu_timer_->Supported()
        << ",\"gpu_disjoint_count\":" << gpu_timer_->DisjointCount() << ",\"physics\":";
    WriteTiming(metrics, physics);
    metrics << ",\"render_cpu\":"; WriteTiming(metrics, render_cpu);
    metrics << ",\"render_gpu\":"; WriteTiming(metrics, render_gpu);
    metrics << "}\n";
    metrics.flush();
    metrics.close();
    std::filesystem::rename(metrics_pending, metrics_final);
    __android_log_print(ANDROID_LOG_INFO, "QuestNewton", "%s run_id=%s states=%llu",
        trace ? "QUEST_NEWTON_TRACE_DONE" : "QUEST_NEWTON_SOAK_DONE", launch_options_.run_id.c_str(),
        static_cast<unsigned long long>(state_count));
}
} // namespace quest_newton
