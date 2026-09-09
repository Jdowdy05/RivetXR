#pragma once
#include "control_boundary.h"
#include "full_control.h"
#include <array>
#include <atomic>
#include <filesystem>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace quest_newton {
std::int64_t InteractionNowNs();
struct InteractionInputObservation {
    kinematics::Pose head;
    bool head_valid=false;
    InputValues raw{};
    FullInputFrame input;
    SimSettings settings;
    std::uint64_t settings_generation=0,input_sequence=0;
};
enum class InteractionObservationKind : unsigned char { Boundary, Clock, Physics, Publication, Render, Command, Loss };
// External observations only: replay does not simulate scheduler, physics or XR.
// Caller defines counter/value meanings in its build's diagnostic documentation.
struct InteractionObservation {
    InteractionObservationKind kind=InteractionObservationKind::Boundary;
    std::array<std::uint64_t,4> counters{};
    std::array<double,4> values{};
    std::uint64_t flags=0;
};
enum class InteractionStopReason : unsigned char { Operator, DurationLimit, EventLimit, ByteLimit, InvalidEvent, SaveFailed };
struct InteractionTraceStatus {
    bool recording=false,pending_flush=false,saved=false,loss=false;
    std::uint32_t event_count=0,control_count=0,input_count=0,observation_count=0;
    InteractionStopReason stop_reason=InteractionStopReason::Operator;
    std::string run_id;
};
class InteractionTrace {
public:
    static constexpr std::uint32_t kMaxEvents=40000;
    static constexpr std::size_t kMaxBytes=32*1024*1024;
    static constexpr std::int64_t kMaxDurationNs=60000000000LL;
    // Only worker Start/Stop/Flush; append is safe from XR and worker threads.
    // Attach observer first, then Start and immediately call control.Reset().
    bool Start(std::string run_id,const SimSettings& settings,std::int64_t now_ns,std::string& error);
    bool AppendControl(const FullControlEvent& event,std::int64_t now_ns) noexcept;
    bool AppendInput(const InteractionInputObservation& event,std::int64_t now_ns) noexcept;
    bool AppendObservation(const InteractionObservation& event,std::int64_t now_ns) noexcept;
    void Stop(std::int64_t now_ns);
    // Outer-worker cooperative deadline, including faulted/no-producer periods.
    void Expire(std::int64_t now_ns);
    InteractionTraceStatus Status() const;
    bool Recording() const noexcept{return active_.load(std::memory_order_acquire);}
    bool NeedsFlush() const;
    // Worker-only; app pauses/flushed clock around saving. Atomic full-file write.
    // Failed preparation or I/O preserves the pending recording for retry.
    bool Flush(const std::filesystem::path& file,std::string& error);
private:
    bool Append(unsigned char kind,std::span<const std::byte> payload,std::int64_t now_ns,std::uint64_t generation);
    void Reject(std::uint64_t generation) noexcept;
    mutable std::mutex mutex_;
    std::vector<std::byte> data_;
    InteractionTraceStatus status_;
    std::int64_t start_ns_=0,end_ns_=0,last_ns_=0;
    bool flushing_=false,seeded_=false;
    std::atomic<bool> active_{false};
    std::atomic<std::uint64_t> generation_{0};
    std::thread::id worker_;
};
struct InteractionReplayResult {
    bool passed=false;
    std::uint32_t event_count=0,control_count=0,input_count=0,external_observations=0;
    std::string run_id,error;
};
// Strict framing/integrity/loss validation followed by exact control-output replay.
InteractionReplayResult ReplayInteractionTrace(std::span<const std::byte> data);
InteractionReplayResult ReplayInteractionFile(const std::filesystem::path& file);
} // namespace quest_newton
