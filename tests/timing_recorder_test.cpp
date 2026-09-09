#include "timing_recorder.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using quest_newton::verification::TimingRecorder;
using quest_newton::verification::TimingSummary;

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance = 1e-12) {
    Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          "timing value mismatch");
}

void EmptyRecorderIsUnavailable() {
    const TimingSummary summary = TimingRecorder(2000.0).Snapshot();
    Check(!summary.available, "empty recorder reported available data");
    Check(summary.count == 0, "empty recorder reported samples");
    Near(summary.mean_us, 0.0);
    Near(summary.p95_us, 0.0);
    Near(summary.p99_us, 0.0);
    Near(summary.max_us, 0.0);
    Check(summary.over_budget_count == 0, "empty recorder reported a budget miss");
}

void KnownSamplesUseNearestRankAndStrictBudget() {
    TimingRecorder recorder(2000.0);
    for (int sample = 0; sample < 95; ++sample) {
        Check(recorder.Observe(1000.0), "valid sample rejected");
    }
    for (int sample = 0; sample < 4; ++sample) {
        Check(recorder.Observe(2000.0), "valid sample rejected");
    }
    Check(recorder.Observe(5000.0), "valid sample rejected");

    const TimingSummary summary = recorder.Snapshot();
    Check(summary.available, "nonempty recorder unavailable");
    Check(summary.count == 100, "known sample count mismatch");
    Near(summary.mean_us, 1080.0);
    Near(summary.p95_us, 1000.0);
    Near(summary.p99_us, 2000.0);
    Near(summary.max_us, 5000.0);
    Check(summary.over_budget_count == 1, "budget comparison was not strict");
}

void SingleZeroDurationIsAValidObservation() {
    TimingRecorder recorder(0.0);
    Check(recorder.Observe(0.0), "zero duration rejected");
    const TimingSummary summary = recorder.Snapshot();
    Check(summary.available && summary.count == 1, "single sample missing");
    Near(summary.mean_us, 0.0);
    Near(summary.p95_us, 0.0);
    Near(summary.p99_us, 0.0);
    Near(summary.max_us, 0.0);
    Check(summary.over_budget_count == 0, "zero duration exceeded zero budget");
}

void PercentilesAreConservativeMicrosecondUpperBounds() {
    TimingRecorder recorder(10.0);
    for (int sample = 0; sample < 95; ++sample) {
        Check(recorder.Observe(1.01), "valid sample rejected");
    }
    for (int sample = 0; sample < 5; ++sample) {
        Check(recorder.Observe(2.01), "valid sample rejected");
    }

    const TimingSummary summary = recorder.Snapshot();
    Check(summary.count == 100, "quantized sample count mismatch");
    Near(summary.mean_us, 1.06);
    Near(summary.p95_us, 2.0);
    Near(summary.p99_us, 2.01);
    Near(summary.max_us, 2.01);
}

void OverflowPercentileReportsTheObservedMaximum() {
    TimingRecorder recorder(20000.0);
    for (int sample = 0; sample < 95; ++sample) {
        Check(recorder.Observe(100.0), "valid sample rejected");
    }
    for (int sample = 0; sample < 5; ++sample) {
        Check(recorder.Observe(25000.0), "valid sample rejected");
    }

    const TimingSummary summary = recorder.Snapshot();
    Near(summary.p95_us, 100.0);
    Near(summary.p99_us, 25000.0);
    Near(summary.max_us, 25000.0);
    Check(summary.over_budget_count == 5, "overflow samples did not count against budget");
}

void QuantizedPercentilesDoNotExceedTheObservedMaximum() {
    TimingRecorder recorder(1000.0);
    for (int sample = 0; sample < 100; ++sample) {
        Check(recorder.Observe(100.1), "valid sample rejected");
    }

    const TimingSummary summary = recorder.Snapshot();
    Near(summary.p95_us, 100.1);
    Near(summary.p99_us, 100.1);
    Near(summary.max_us, 100.1);
}

void InvalidValuesAndSumOverflowPreserveState() {
    TimingRecorder recorder(std::numeric_limits<double>::max());
    Check(recorder.Observe(std::numeric_limits<double>::max()), "finite duration rejected");
    Check(!recorder.Observe(-1.0), "negative duration accepted");
    Check(!recorder.Observe(std::numeric_limits<double>::quiet_NaN()), "NaN duration accepted");
    Check(!recorder.Observe(std::numeric_limits<double>::infinity()), "infinite duration accepted");
    Check(!recorder.Observe(std::numeric_limits<double>::max()), "overflowing sum accepted");

    const TimingSummary summary = recorder.Snapshot();
    Check(summary.available && summary.count == 1, "rejected observations changed count");
    Near(summary.mean_us, std::numeric_limits<double>::max());
    Near(summary.p95_us, std::numeric_limits<double>::max());
    Near(summary.p99_us, std::numeric_limits<double>::max());
    Near(summary.max_us, std::numeric_limits<double>::max());
    Check(summary.over_budget_count == 0, "rejected observations changed budget count");
}

void InvalidBudgetsAreRejected() {
    for (double budget : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
        bool threw = false;
        try {
            TimingRecorder recorder(budget);
            (void)recorder;
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        Check(threw, "invalid timing budget accepted");
    }
}

void ConcurrentSnapshotsAreCoherentAndKeepEveryObservation() {
    TimingRecorder recorder(100.0);
    constexpr int kThreadCount = 4;
    constexpr int kSamplesPerThread = 2500;
    std::atomic<bool> start{false};
    std::atomic<int> first_observations{0};
    std::atomic<bool> first_snapshot_complete{false};
    std::atomic<int> finished{0};
    std::atomic<bool> accepted{true};
    std::vector<std::thread> writers;
    writers.reserve(kThreadCount);
    for (int thread = 0; thread < kThreadCount; ++thread) {
        writers.emplace_back([&] {
            while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
            if (!recorder.Observe(100.0)) accepted.store(false, std::memory_order_relaxed);
            first_observations.fetch_add(1, std::memory_order_release);
            while (!first_snapshot_complete.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            for (int sample = 1; sample < kSamplesPerThread; ++sample) {
                if (!recorder.Observe(100.0)) accepted.store(false, std::memory_order_relaxed);
            }
            finished.fetch_add(1, std::memory_order_release);
        });
    }

    start.store(true, std::memory_order_release);
    while (first_observations.load(std::memory_order_acquire) != kThreadCount) {
        std::this_thread::yield();
    }
    const TimingSummary first_summary = recorder.Snapshot();
    first_snapshot_complete.store(true, std::memory_order_release);
    bool coherent = first_summary.available && first_summary.count == kThreadCount &&
        first_summary.mean_us == 100.0 && first_summary.p95_us == 100.0 &&
        first_summary.p99_us == 100.0 && first_summary.max_us == 100.0 &&
        first_summary.over_budget_count == 0;
    std::uint64_t prior_count = first_summary.count;
    while (finished.load(std::memory_order_acquire) != kThreadCount) {
        const TimingSummary summary = recorder.Snapshot();
        coherent &= summary.count >= prior_count;
        coherent &= summary.count <= kThreadCount * kSamplesPerThread;
        coherent &= summary.available == (summary.count != 0);
        if (summary.available) {
            coherent &= summary.mean_us == 100.0;
            coherent &= summary.p95_us == 100.0;
            coherent &= summary.p99_us == 100.0;
            coherent &= summary.max_us == 100.0;
            coherent &= summary.over_budget_count == 0;
        }
        prior_count = summary.count;
    }
    for (auto& writer : writers) writer.join();

    const TimingSummary summary = recorder.Snapshot();
    Check(accepted.load(std::memory_order_relaxed), "concurrent valid observation rejected");
    Check(coherent, "concurrent snapshot was internally inconsistent");
    Check(summary.count == 10000, "concurrent observations were lost");
    Near(summary.mean_us, 100.0);
    Near(summary.p95_us, 100.0);
    Near(summary.p99_us, 100.0);
    Near(summary.max_us, 100.0);
    Check(summary.over_budget_count == 0, "concurrent budget count mismatch");
}
}  // namespace

int main() {
    try {
        EmptyRecorderIsUnavailable();
        TimingRecorder resettable(10);
        Check(resettable.Observe(50), "pre-reset observation");
        resettable.Reset();
        Check(!resettable.Snapshot().available, "reset retained warmup");
        Check(resettable.Observe(5), "post-reset observation");
        Check(resettable.Snapshot().count == 1 && resettable.Snapshot().mean_us == 5 &&
            resettable.Snapshot().over_budget_count == 0, "reset did not clear all counters");
        KnownSamplesUseNearestRankAndStrictBudget();
        SingleZeroDurationIsAValidObservation();
        PercentilesAreConservativeMicrosecondUpperBounds();
        OverflowPercentileReportsTheObservedMaximum();
        QuantizedPercentilesDoNotExceedTheObservedMaximum();
        InvalidValuesAndSumOverflowPreserveState();
        InvalidBudgetsAreRejected();
        ConcurrentSnapshotsAreCoherentAndKeepEveryObservation();
        std::puts("timing recorder tests passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "timing recorder test failed: %s\n", error.what());
        return 1;
    }
}
