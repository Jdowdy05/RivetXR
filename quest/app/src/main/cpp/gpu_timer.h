#pragma once

#include "timing_recorder.h"

#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace quest_newton::verification {

class GpuTimer {
public:
    explicit GpuTimer(TimingRecorder& recorder);

    bool Init();
    void Begin();
    void End(bool record_this_frame);
    void Shutdown();

    bool Supported() const;
    std::uint64_t DisjointCount() const;

private:
    static constexpr std::size_t kQueryCount = 8;
    static constexpr std::size_t kNoQuery = kQueryCount;

    enum class ExtensionState {
        kUnavailable,
        kAvailable,
        kError,
    };

    ExtensionState ExtensionSupported() const;
    void PollQueries();
    void MarkRuntimeFailure(const char* stage);
    bool ReportGlErrors(const char* stage) const;

    TimingRecorder& recorder_;
    std::array<GLuint, kQueryCount> queries_{};
    std::array<bool, kQueryCount> pending_{};
    std::array<bool, kQueryCount> record_result_{};
    PFNGLGETQUERYOBJECTUI64VEXTPROC get_query_result_ = nullptr;
    std::atomic<bool> supported_{false};
    std::atomic<std::uint64_t> disjoint_count_{0};
    std::size_t active_query_ = kNoQuery;
    bool initialized_ = false;
    bool queries_allocated_ = false;
    bool frame_open_ = false;
};

}  // namespace quest_newton::verification
