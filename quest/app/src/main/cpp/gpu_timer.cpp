#include "gpu_timer.h"

#include <EGL/egl.h>
#include <android/log.h>

#include <cmath>
#include <cstring>

namespace quest_newton::verification {
namespace {

constexpr char kLogTag[] = "QuestNewton";

void LogFailure(const char* stage, const char* detail) {
    __android_log_print(ANDROID_LOG_ERROR, kLogTag,
        "QUEST_NEWTON_VERIFY_FAIL stage=gpu_timer_%s detail=%s", stage, detail);
}

}  // namespace

GpuTimer::GpuTimer(TimingRecorder& recorder) : recorder_(recorder) {}

bool GpuTimer::Init() {
    if (initialized_) return true;

    supported_.store(false, std::memory_order_release);
    if (ReportGlErrors("init_preexisting")) return false;

    const ExtensionState extension = ExtensionSupported();
    if (extension == ExtensionState::kError) return false;
    if (extension == ExtensionState::kUnavailable) {
        initialized_ = true;
        return true;
    }

    GLint timer_bits = 0;
    glGetQueryiv(GL_TIME_ELAPSED_EXT, GL_QUERY_COUNTER_BITS_EXT, &timer_bits);
    if (ReportGlErrors("counter_bits")) return false;
    if (timer_bits <= 0) {
        initialized_ = true;
        return true;
    }

    get_query_result_ = reinterpret_cast<PFNGLGETQUERYOBJECTUI64VEXTPROC>(
        eglGetProcAddress("glGetQueryObjectui64vEXT"));
    if (get_query_result_ == nullptr) {
        LogFailure("dispatch", "glGetQueryObjectui64vEXT_unavailable");
        return false;
    }

    glGenQueries(static_cast<GLsizei>(queries_.size()), queries_.data());
    if (ReportGlErrors("allocate_queries")) {
        glDeleteQueries(static_cast<GLsizei>(queries_.size()), queries_.data());
        ReportGlErrors("cleanup_failed_allocation");
        queries_.fill(0);
        get_query_result_ = nullptr;
        return false;
    }
    for (GLuint query : queries_) {
        if (query == 0) {
            LogFailure("allocate_queries", "zero_query_name");
            glDeleteQueries(static_cast<GLsizei>(queries_.size()), queries_.data());
            ReportGlErrors("cleanup_zero_query");
            queries_.fill(0);
            get_query_result_ = nullptr;
            return false;
        }
    }

    queries_allocated_ = true;
    initialized_ = true;
    supported_.store(true, std::memory_order_release);
    return true;
}

void GpuTimer::Begin() {
    if (!initialized_) {
        LogFailure("begin", "not_initialized");
        return;
    }
    if (!supported_.load(std::memory_order_acquire)) return;
    if (frame_open_) {
        MarkRuntimeFailure("begin_while_frame_open");
        return;
    }

    frame_open_ = true;
    PollQueries();
    if (!supported_.load(std::memory_order_acquire)) return;

    std::size_t slot = kNoQuery;
    for (std::size_t index = 0; index < pending_.size(); ++index) {
        if (!pending_[index]) {
            slot = index;
            break;
        }
    }
    if (slot == kNoQuery) return;

    glBeginQuery(GL_TIME_ELAPSED_EXT, queries_[slot]);
    if (ReportGlErrors("begin_query")) {
        supported_.store(false, std::memory_order_release);
        return;
    }
    active_query_ = slot;
}

void GpuTimer::End(bool record_this_frame) {
    if (!initialized_) {
        LogFailure("end", "not_initialized");
        return;
    }
    if (!frame_open_) {
        if (supported_.load(std::memory_order_acquire)) {
            MarkRuntimeFailure("end_without_begin");
        }
        return;
    }

    frame_open_ = false;
    if (active_query_ == kNoQuery) return;

    const std::size_t slot = active_query_;
    active_query_ = kNoQuery;
    glEndQuery(GL_TIME_ELAPSED_EXT);
    if (ReportGlErrors("end_query")) {
        record_result_[slot] = false;
        pending_[slot] = true;
        supported_.store(false, std::memory_order_release);
        return;
    }

    record_result_[slot] = record_this_frame;
    pending_[slot] = true;
}

void GpuTimer::Shutdown() {
    if (active_query_ != kNoQuery) {
        glEndQuery(GL_TIME_ELAPSED_EXT);
        ReportGlErrors("shutdown_end_query");
        active_query_ = kNoQuery;
    }
    frame_open_ = false;

    if (queries_allocated_) {
        glDeleteQueries(static_cast<GLsizei>(queries_.size()), queries_.data());
        ReportGlErrors("delete_queries");
        queries_allocated_ = false;
    }

    queries_.fill(0);
    pending_.fill(false);
    record_result_.fill(false);
    get_query_result_ = nullptr;
    supported_.store(false, std::memory_order_release);
    initialized_ = false;
}

bool GpuTimer::Supported() const {
    return supported_.load(std::memory_order_acquire);
}

std::uint64_t GpuTimer::DisjointCount() const {
    return disjoint_count_.load(std::memory_order_acquire);
}

GpuTimer::ExtensionState GpuTimer::ExtensionSupported() const {
    GLint extension_count = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &extension_count);
    if (ReportGlErrors("extension_count") || extension_count < 0) {
        if (extension_count < 0) LogFailure("extension_count", "negative_count");
        return ExtensionState::kError;
    }

    for (GLint index = 0; index < extension_count; ++index) {
        const GLubyte* extension = glGetStringi(GL_EXTENSIONS, static_cast<GLuint>(index));
        if (extension == nullptr) {
            ReportGlErrors("extension_name");
            LogFailure("extension_name", "null_name");
            return ExtensionState::kError;
        }
        if (std::strcmp(reinterpret_cast<const char*>(extension),
                        "GL_EXT_disjoint_timer_query") == 0) {
            return ReportGlErrors("extension_name")
                ? ExtensionState::kError
                : ExtensionState::kAvailable;
        }
    }

    return ReportGlErrors("extension_names")
        ? ExtensionState::kError
        : ExtensionState::kUnavailable;
}

void GpuTimer::PollQueries() {
    GLint disjoint = GL_FALSE;
    glGetIntegerv(GL_GPU_DISJOINT_EXT, &disjoint);
    if (ReportGlErrors("disjoint_status")) {
        supported_.store(false, std::memory_order_release);
        return;
    }
    if (disjoint != GL_FALSE) {
        disjoint_count_.fetch_add(1, std::memory_order_relaxed);
        for (std::size_t index = 0; index < pending_.size(); ++index) {
            if (pending_[index]) record_result_[index] = false;
        }
    }

    for (std::size_t index = 0; index < pending_.size(); ++index) {
        if (!pending_[index]) continue;

        GLuint available = GL_FALSE;
        glGetQueryObjectuiv(queries_[index], GL_QUERY_RESULT_AVAILABLE, &available);
        if (ReportGlErrors("result_available")) {
            supported_.store(false, std::memory_order_release);
            return;
        }
        if (available == GL_FALSE) continue;

        if (record_result_[index]) {
            GLuint64 duration_ns = 0;
            get_query_result_(queries_[index], GL_QUERY_RESULT, &duration_ns);
            if (ReportGlErrors("query_result")) {
                supported_.store(false, std::memory_order_release);
                return;
            }
            const double duration_us = static_cast<double>(duration_ns) / 1000.0;
            if (!std::isfinite(duration_us) || !recorder_.Observe(duration_us)) {
                LogFailure("record_result", "invalid_or_rejected_duration");
            }
        }

        record_result_[index] = false;
        pending_[index] = false;
    }
}

void GpuTimer::MarkRuntimeFailure(const char* stage) {
    LogFailure(stage, "invalid_lifecycle");
    supported_.store(false, std::memory_order_release);
}

bool GpuTimer::ReportGlErrors(const char* stage) const {
    bool found_error = false;
    for (GLenum error = glGetError(); error != GL_NO_ERROR; error = glGetError()) {
        found_error = true;
        __android_log_print(ANDROID_LOG_ERROR, kLogTag,
            "QUEST_NEWTON_VERIFY_FAIL stage=gpu_timer_%s gl_error=0x%x",
            stage, static_cast<unsigned int>(error));
    }
    return found_error;
}

}  // namespace quest_newton::verification
