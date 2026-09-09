#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>

namespace quest_newton::verification {

struct TimingSummary {
    bool available = false;
    std::uint64_t count = 0;
    double mean_us = 0.0;
    double p95_us = 0.0;
    double p99_us = 0.0;
    double max_us = 0.0;
    std::uint64_t over_budget_count = 0;
};

class TimingRecorder {
public:
    explicit TimingRecorder(double budget_us) : budget_us_(budget_us) {
        if (!std::isfinite(budget_us) || budget_us < 0.0) {
            throw std::invalid_argument("timing budget must be finite and nonnegative");
        }
    }

    bool Observe(double microseconds) {
        if (!std::isfinite(microseconds) || microseconds < 0.0) return false;

        const std::size_t bucket = microseconds >= static_cast<double>(kOverflowBucket)
            ? kOverflowBucket
            : static_cast<std::size_t>(std::ceil(microseconds));

        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == std::numeric_limits<std::uint64_t>::max()) return false;
        if (histogram_[bucket] == std::numeric_limits<std::uint64_t>::max()) return false;
        if (microseconds > std::numeric_limits<double>::max() - sum_us_) return false;

        const double updated_sum = sum_us_ + microseconds;
        if (!std::isfinite(updated_sum)) return false;

        ++histogram_[bucket];
        ++count_;
        sum_us_ = updated_sum;
        if (microseconds > max_us_) max_us_ = microseconds;
        if (microseconds > budget_us_) ++over_budget_count_;
        return true;
    }

    // Preserve the recorder's identity: the render/GPU thread holds references.
    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        histogram_.fill(0);
        count_ = 0; sum_us_ = 0; max_us_ = 0; over_budget_count_ = 0;
    }

    TimingSummary Snapshot() const {
        std::array<std::uint64_t, kBucketCount> histogram;
        TimingSummary summary;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            histogram = histogram_;
            summary.count = count_;
            summary.mean_us = count_ == 0 ? 0.0 : sum_us_ / static_cast<double>(count_);
            summary.max_us = max_us_;
            summary.over_budget_count = over_budget_count_;
        }

        if (summary.count == 0) return summary;

        summary.available = true;
        summary.p95_us = PercentileUpperBound(histogram, summary.count, 95, summary.max_us);
        summary.p99_us = PercentileUpperBound(histogram, summary.count, 99, summary.max_us);
        return summary;
    }

private:
    static constexpr std::size_t kOverflowBucket = 20000;
    static constexpr std::size_t kBucketCount = kOverflowBucket + 1;

    static std::uint64_t NearestRank(std::uint64_t count, std::uint64_t percent) {
        const std::uint64_t whole_hundreds = count / 100;
        const std::uint64_t remainder = count % 100;
        return whole_hundreds * percent + (remainder * percent + 99) / 100;
    }

    static double PercentileUpperBound(
        const std::array<std::uint64_t, kBucketCount>& histogram,
        std::uint64_t count,
        std::uint64_t percent,
        double observed_max) {
        const std::uint64_t rank = NearestRank(count, percent);
        std::uint64_t cumulative = 0;
        for (std::size_t bucket = 0; bucket < histogram.size(); ++bucket) {
            cumulative += histogram[bucket];
            if (cumulative >= rank) {
                const double bucket_upper_bound = static_cast<double>(bucket);
                return bucket == kOverflowBucket || observed_max < bucket_upper_bound
                    ? observed_max
                    : bucket_upper_bound;
            }
        }
        return observed_max;
    }

    const double budget_us_;
    mutable std::mutex mutex_;
    std::array<std::uint64_t, kBucketCount> histogram_{};
    std::uint64_t count_ = 0;
    double sum_us_ = 0.0;
    double max_us_ = 0.0;
    std::uint64_t over_budget_count_ = 0;
};

}  // namespace quest_newton::verification
