#include "full_benchmark.h"
#include "quest_newton_app.h"
#include "full_runtime_state.h"
#include "python_session.h"
#include "simulation_clock.h"
#include <android/log.h>
#include <android/trace.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace quest_newton {
namespace {
namespace kin = kinematics;
double Now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string Quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (const unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
        else out << static_cast<char>(c);
    }
    out << '"';
    return out.str();
}
void WriteAtomic(const std::filesystem::path& path, const std::string& text) {
    auto pending = path; pending += ".pending";
    std::ofstream file(pending, std::ios::binary | std::ios::trunc);
    file << text;
    file.close();
    if (!file) throw std::runtime_error("benchmark report write failed");
    std::filesystem::rename(pending, path);
}
SceneSnapshot DecodeBenchmark(const std::vector<std::byte>& bytes) {
    SceneSnapshot snapshot;
    std::string error;
    if (!DecodeSceneSnapshot(bytes, snapshot, error)) throw std::runtime_error("benchmark snapshot: " + error);
    if (snapshot.objects.size() != 1) throw std::runtime_error("benchmark requires exactly one debug box");
    return snapshot;
}
std::string InitialSettings(const SimSettings& settings, const kin::Pose& base) {
    auto json = SettingsJson(settings); json.pop_back();
    std::ostringstream out;
    out.imbue(std::locale::classic()); out << std::setprecision(17);
    out << json << ",\"initial_base_pose\":[";
    for (std::size_t i = 0; i < 3; ++i) out << (i ? "," : "") << base.position[i];
    for (float value : base.rotation) out << ',' << value;
    out << "]}";
    return out.str();
}
std::string TimingJson(const verification::TimingSummary& value) {
    std::ostringstream out;
    out.imbue(std::locale::classic()); out << std::setprecision(17) << std::boolalpha;
    out << "{\"available\":" << value.available << ",\"count\":" << value.count;
    if (value.available) {
        out << ",\"mean_us\":" << value.mean_us << ",\"p95_us\":" << value.p95_us
            << ",\"p99_us\":" << value.p99_us << ",\"max_us\":" << value.max_us;
    } else {
        out << ",\"mean_us\":null,\"p95_us\":null,\"p99_us\":null,\"max_us\":null";
    }
    out << ",\"over_budget_count\":" << value.over_budget_count << '}';
    return out.str();
}
struct Trace {
    explicit Trace(const char* name) { ATrace_beginSection(name); }
    ~Trace() { ATrace_endSection(); }
};
} // namespace

void QuestNewtonApp::RunFullBenchmark() {
    auto& f = *full_;
    const auto& options = launch_options_;
    const auto settings = BenchmarkSettings(options);
    const bool motion = options.benchmark_workload == "motion";
    const auto root = std::filesystem::path(internal_data_path_);
    const double worker_started = Now();
    double warmup_start = 0, measure_start = 0, measure_end = 0;
    double sim_start = 0, wall_seconds = 0, last_report = 0;
    std::uint64_t steps = 0, controls = 0, publications = 0, frame_start = 0, frame_count = 0;
    std::uint64_t reference = 0, ik_converged = 0;
    std::uint64_t gpu_disjoint_start = 0, gpu_disjoint_count = 0;
    float refresh = 0;
    bool gpu_supported = false;
    bool finished = false, measuring = false, finite = false;
    bool warmup_trace = false, measure_trace = false;
    std::string phase = "waiting", failure;
    SimulationClock clock;
    SceneSnapshot latest, initial;
    kin::Pose base;
    kin::JointVector targets = kin::kHome;
    float gripper = 0;
    double box_initial = 0, box_min = 0, box_final = 0;
    double target_joint_motion = 0, body_motion = 0;
    std::uint64_t floor_samples = 0;
    std::uint32_t contact_min = 0, contact_max = 0;
    // Histograms are large; keep the Android/Python worker stack available for
    // the interpreter and solver calls, as the normal app does for its timers.
    const auto physics_owner = std::make_unique<verification::TimingRecorder>(1e6 / options.benchmark_hz);
    const auto ik_owner = std::make_unique<verification::TimingRecorder>(1e6 / settings.ControlHz());
    const auto base_owner = std::make_unique<verification::TimingRecorder>(1e6 / settings.ControlHz());
    auto& physics = *physics_owner;
    auto& ik = *ik_owner;
    auto& base_timing = *base_owner;
    verification::TimingSummary cpu_final, gpu_final;
    std::unique_ptr<PythonSession> session;
    const auto status_json = [&] {
        std::ostringstream out;
        out.imbue(std::locale::classic()); out << std::setprecision(17) << std::boolalpha;
        out << "{\"schema_version\":1,\"run_id\":" << Quote(options.run_id)
            << ",\"pid\":" << getpid() << ",\"mode\":\"benchmark\",\"backend\":\"Newton SolverMuJoCo CPU\""
            << ",\"requested_physics_hz\":" << options.benchmark_hz << ",\"workload\":" << Quote(options.benchmark_workload)
            << ",\"phase\":" << Quote(phase) << ",\"worker_started\":" << worker_started << ",\"monotonic_s\":" << Now()
            << ",\"warmup_started_monotonic_s\":" << warmup_start
            << ",\"measure_started_monotonic_s\":" << measure_start
            << ",\"measure_ended_monotonic_s\":" << measure_end
            << ",\"requested_seconds\":" << options.benchmark_seconds
            << ",\"warmup_seconds\":" << options.benchmark_warmup_seconds
            << ",\"configured_physics_hz\":" << 1.0 / settings.physics_dt
            << ",\"configured_control_hz\":" << settings.ControlHz()
            << ",\"configured_publication_hz\":" << settings.SceneHz()
            << ",\"physics_dt\":" << settings.physics_dt
            << ",\"control_decimation\":" << settings.control_decimation
            << ",\"render_interval\":" << settings.render_interval
            << ",\"failure\":" << Quote(failure);
        if (finished) {
            const double sim_seconds = measure_start > 0 ? latest.simulation_time - sim_start : 0;
            const bool pipeline = finite && steps > 0 && frame_count > 0;
            out << ",\"finite\":" << finite << ",\"passed\":" << (phase == "complete" && pipeline)
                << ",\"pass_scope\":\"finite pipeline completion; useful rate and frame delivery require host analysis\""
                << ",\"wall_seconds\":" << wall_seconds << ",\"simulation_seconds\":" << sim_seconds
                << ",\"successful_steps\":" << steps << ",\"dropped_wall_seconds\":" << clock.DroppedWallSeconds()
                << ",\"control_updates\":" << controls << ",\"publications\":" << publications
                << ",\"achieved_physics_hz\":" << (wall_seconds > 0 ? steps / wall_seconds : 0)
                << ",\"achieved_control_hz\":" << (wall_seconds > 0 ? controls / wall_seconds : 0)
                << ",\"achieved_publication_hz\":" << (wall_seconds > 0 ? publications / wall_seconds : 0)
                << ",\"frame_count\":" << frame_count << ",\"refresh_hz\":" << refresh
                << ",\"physics_timing\":" << TimingJson(physics.Snapshot())
                << ",\"ik_timing\":" << TimingJson(ik.Snapshot())
                << ",\"base_timing\":" << TimingJson(base_timing.Snapshot())
                << ",\"xr_cpu_timing\":" << TimingJson(cpu_final)
                << ",\"gpu_timing\":" << TimingJson(gpu_final)
                << ",\"gpu_supported\":" << gpu_supported << ",\"gpu_disjoint_count\":" << gpu_disjoint_count
                << ",\"ik_converged_updates\":" << ik_converged
                << ",\"max_target_joint_delta_rad\":" << target_joint_motion
                << ",\"max_robot_body_displacement_m\":" << body_motion
                << ",\"box_initial_z\":" << box_initial << ",\"box_min_z\":" << box_min << ",\"box_final_z\":" << box_final
                << ",\"global_contact_min\":" << contact_min << ",\"global_contact_max\":" << contact_max
                << ",\"box_floor_height_samples\":" << floor_samples
                << ",\"box_settled_floor_observed\":" << (floor_samples > 10 && std::abs(box_final - .05) < .015)
                << ",\"contact_scope\":\"box height and global contacts; no box-specific contact attribution\""
                << ",\"render_boundary_note\":\"CPU frame may straddle reset; asynchronous GPU queries may straddle either boundary; use exact ATrace interval for frame-delivery analysis\"";
        }
        out << '}'; return out.str();
    };
    const auto write_status = [&] { WriteAtomic(root / "full-benchmark-status.json", status_json()); last_report = Now(); };
    const auto close_trace = [&] {
        if (measure_trace) { ATrace_endSection(); measure_trace = false; }
        if (warmup_trace) { ATrace_endSection(); warmup_trace = false; }
    };
    const auto finish = [&](const std::string& why) {
        if (measuring) {
            measure_end = Now(); wall_seconds = measure_end - measure_start;
            close_trace();
            frame_count = render_frames_.load() - frame_start;
            cpu_final = render_cpu_timing_->Snapshot(); gpu_final = render_gpu_timing_->Snapshot();
            gpu_supported = gpu_timer_->Supported();
            gpu_disjoint_count = gpu_timer_->DisjointCount() - gpu_disjoint_start;
        }
        close_trace(); verification_running_.store(false);
        failure = why; phase = why.empty() ? "complete" : "failed";
        finished = true; measuring = false;
        write_status();
        WriteAtomic(root / ("full-benchmark-" + options.run_id + ".json"), status_json());
        __android_log_print(why.empty() ? ANDROID_LOG_INFO : ANDROID_LOG_ERROR, "QuestNewton",
            "QUEST_FULL_BENCHMARK run_id=%s phase=%s steps=%llu failure=%s", options.run_id.c_str(), phase.c_str(),
            static_cast<unsigned long long>(steps), why.c_str());
    };
    try { write_status(); } catch (const std::exception& error) { Fail("benchmark_report", error.what()); return; }
    while (!stop_worker_.load()) {
        try {
            FullInputFrame input;
            bool fault;
            { const std::lock_guard lock(f.mutex); input = f.input; fault = f.fault; }
            const double now = Now();
            const auto age = std::chrono::steady_clock::now() - input.sampled;
            const bool valid = input.registered && input.controller.focused && input.controller.stage_valid &&
                age >= std::chrono::steady_clock::duration::zero() && age < std::chrono::milliseconds(100) &&
                std::isfinite(refresh_hz_.load()) && std::abs(refresh_hz_.load() - 90.F) < .1F;
            if (!finished && fault) throw std::runtime_error("full runtime reported a fault");
            if (!finished && !session && valid && verification_frame_available_.load()) {
                base = input.world_from_base; reference = input.reference;
                session = std::make_unique<PythonSession>(f.vm, f.activity, f.bridge, InitialSettings(settings, base));
                latest = DecodeBenchmark(session->Command("spawn_box"));
                f.scenes.Publish(latest);
                __android_log_print(ANDROID_LOG_INFO, "QuestNewton", "QUEST_FULL_BENCHMARK_RUNTIME %s", session->Metadata().c_str());
                if (!clock.Configure(settings, Now())) throw std::runtime_error("benchmark clock configuration invalid");
                clock.Reset(Now());
                warmup_start = Now(); phase = "warmup"; write_status();
                ATrace_beginSection("QuestFull.BenchmarkWarmup"); warmup_trace = true;
            } else if (!finished && !session && now - worker_started > 180) {
                throw std::runtime_error("timed out waiting for focused tracked STAGE render frame");
            }
            if (!finished && session) {
                if (measuring && (!valid || input.reference != reference || verification_interrupted_.load()))
                    throw std::runtime_error("focus, STAGE/head tracking, refresh or render-input validity lost during measurement");
                // Even a zero-second warmup must run Step once: scene creation
                // and rendering alone cannot exclude first-step solver/JIT work.
                if (!measuring && latest.step_index > 0 &&
                    Now() - warmup_start >= options.benchmark_warmup_seconds && valid && render_frames_.load() > 0) {
                    close_trace();
                    latest = DecodeBenchmark(session->SetBasePose(base));
                    latest = DecodeBenchmark(session->Command("reset"));
                    initial = latest;
                    targets = kin::kHome; gripper = 0;
                    steps = controls = publications = ik_converged = floor_samples = 0;
                    target_joint_motion = body_motion = 0;
                    box_initial = box_min = box_final = latest.bodies[latest.objects.front().body_index].position[2];
                    contact_min = contact_max = latest.contact_count;
                    sim_start = latest.simulation_time;
                    if (latest.step_index != 0 || sim_start != 0) throw std::runtime_error("scene reset did not clear physics counters");
                    f.scenes.Publish(latest);
                    physics.Reset(); ik.Reset(); base_timing.Reset();
                    finite = true; refresh = refresh_hz_.load();
                    phase = "measuring"; write_status();
                    render_cpu_timing_->Reset(); render_gpu_timing_->Reset();
                    verification_interrupted_.store(false); verification_running_.store(true);
                    frame_start = render_frames_.load();
                    gpu_disjoint_start = gpu_timer_->DisjointCount();
                    measure_start = Now(); clock.Reset(measure_start);
                    ATrace_beginSection("QuestFull.BenchmarkMeasure"); measure_trace = true; measuring = true;
                }
                if (measuring && Now() - measure_start >= options.benchmark_seconds) {
                    finish(steps == 0 || render_frames_.load() == frame_start ? "no measured physics steps or rendered frames" : "");
                }
                if (!finished) {
                    if (!measuring && Now() - warmup_start > options.benchmark_warmup_seconds + 180)
                        throw std::runtime_error("timed out waiting for measured render/tracking readiness after warmup");
                    const unsigned due = clock.Accumulate(Now(), false);
                    for (unsigned i = 0; i < due && !stop_worker_.load(); ++i) {
                        if (measuring && Now() - measure_start >= options.benchmark_seconds) break;
                        const auto tick = clock.Advance();
                        if (tick.control_due) {
                            const auto command = BenchmarkWorkload(tick.simulation_time, motion);
                            if (motion) {
                                const auto start = Now();
                                { Trace trace("QuestFull.BenchmarkIK");
                                  const auto result = kin::SolveIK(targets, command.hand);
                                  if (!result.valid) throw std::runtime_error("benchmark IK returned invalid targets");
                                  targets = result.joints;
                                  if (measuring && result.converged) ++ik_converged; }
                                if (measuring) ik.Observe((Now() - start) * 1e6);
                                auto moved_base = base; moved_base.position[2] += command.base_height_offset;
                                const auto base_start = Now();
                                { Trace trace("QuestFull.BenchmarkBase"); latest = DecodeBenchmark(session->SetBasePose(moved_base)); }
                                if (measuring) base_timing.Observe((Now() - base_start) * 1e6);
                            }
                            gripper = command.gripper;
                            if (measuring) {
                                ++controls;
                                for (std::size_t j = 0; j < targets.size(); ++j)
                                    target_joint_motion = std::max(target_joint_motion, static_cast<double>(std::abs(targets[j] - kin::kHome[j])));
                            }
                        }
                        const double start = Now();
                        { Trace trace("QuestFull.BenchmarkStep"); latest = DecodeBenchmark(session->Step(tick.physics_dt, targets, gripper,true)); }
                        if (measuring) {
                            physics.Observe((Now() - start) * 1e6); ++steps;
                            box_final = latest.bodies[latest.objects.front().body_index].position[2];
                            box_min = std::min(box_min, box_final);
                            contact_min = std::min(contact_min, latest.contact_count); contact_max = std::max(contact_max, latest.contact_count);
                            if (latest.simulation_time > 1 && std::abs(box_final - .05) < .015) ++floor_samples;
                            for (std::size_t b = 0; b < kArmBodies; ++b) {
                                double distance2 = 0;
                                for (std::size_t axis = 0; axis < 3; ++axis) {
                                    const double delta = latest.bodies[b].position[axis] - initial.bodies[b].position[axis];
                                    distance2 += delta * delta;
                                }
                                body_motion = std::max(body_motion, std::sqrt(distance2));
                            }
                        }
                        if (tick.publish_due) { f.scenes.Publish(latest); if (measuring) ++publications; }
                    }
                }
            }
            if (!finished && !measuring && Now() - last_report > 1) write_status();
        } catch (const std::exception& error) {
            if (!finished) {
                finite = false;
                try { finish(error.what()); } catch (const std::exception& report_error) { Fail("benchmark_report", report_error.what()); finished = true; }
            }
        } catch (...) {
            if (!finished) {
                finite = false;
                try { finish("unknown benchmark worker failure"); } catch (...) { finished = true; }
            }
        }
        // Keep the same pacing as RunFullSimulation, including overloaded trials.
        // Complete trials remain displayed until the host stops the application.
        std::unique_lock lock(wait_mutex_);
        wake_worker_.wait_for(lock, std::chrono::milliseconds(1), [this] { return stop_worker_.load(); });
    }
    if (!finished) {
        try { finish("application stopped before benchmark completion"); } catch (...) { close_trace(); }
    }
    close_trace(); verification_running_.store(false);
    session.reset();
}
} // namespace quest_newton
