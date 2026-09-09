#pragma once
#include "arm_renderer.h"
#include "passthrough_layer.h"
#include "input_mapper.h"
#include "joint_target_mailbox.h"
#include "launch_options.h"
#include "timing_recorder.h"
#include "verification_state_limits.h"
#include "gpu_timer.h"
#include "XrApp.h"
#include <android/asset_manager.h>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

namespace quest_newton {
class NewtonRuntime;
#if defined(QUEST_NEWTON_FULL_RUNTIME)
struct FullRuntimeState;
enum class SimMenuCommand;
#endif
class QuestNewtonApp final : public OVRFW::XrApp {
  public:
    QuestNewtonApp(AAssetManager* assets, const char* internal_data_path,
                   verification::LaunchOptions options = {});
    ~QuestNewtonApp() override;
  protected:
    std::vector<const char*> GetExtensions() override;
    void GetInitialSceneUri(std::string& uri) const override { uri.clear(); }
    bool AppInit(const xrJava* context) override;
    void AppShutdown(const xrJava* context) override;
    bool SessionInit() override;
    void SessionEnd() override;
    void AttachActionSets() override;
    void SyncActionSets(OVRFW::ovrApplFrameIn& in) override;
    void Update(const OVRFW::ovrApplFrameIn& in) override;
    void SessionStateChanged(XrSessionState state) override;
    void AppHandleEvent(XrEventDataBaseHeader* event) override;
    void AppRenderFrame(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out) override;
    void Render(const OVRFW::ovrApplFrameIn&, OVRFW::ovrRendererOutput& out) override;
    void PreProjectionAddLayer(xrCompositorLayerUnion* layers, int& layer_count) override;
  private:
    void Fail(const char* stage, const std::string& detail);
    bool NativeLibraryDirectory(std::filesystem::path& directory);
    void RunSmoke();
    void RunVerification(NewtonRuntime& simulation);
    bool TimedStep(NewtonRuntime& simulation, bool check_limits = false);
    bool ValidateVerificationState(const NewtonRuntime& simulation, std::string& detail);
    void StopWorker();
    bool ResolvePassthrough();
    bool ConfigureRefreshRate();
    bool CheckInputResult(const char* operation, XrResult result);
    void PublishInput(const ControllerSample& input);
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    bool InitFullRuntime(const xrJava* context);
    void ShutdownFullRuntime(const xrJava* context);
    void RunFullSimulation();
    void RunFullBenchmark();
    void SyncFullInput(OVRFW::ovrApplFrameIn& in);
    void UpdateFullRuntime(const OVRFW::ovrApplFrameIn& in);
    void UpdateRoomEnvironment(const OVRFW::ovrApplFrameIn& in);
    void RenderFullRuntime(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out);
    void SuspendFullInput(bool reference_changed);
    void InitRemoteScene(const xrJava* context);
    void InitRemoteSceneSession();
    void StopRemoteSceneSession(const xrJava* context);
    bool HandleRemoteCommand(SimMenuCommand command);
    void UpdateRemoteScene();
    void UpdateGimbalControl();
    void ResetRemoteTiming();
    bool RenderRemoteScene(const OVRFW::ovrApplFrameIn& in,OVRFW::ovrRendererOutput& out);
    std::unique_ptr<FullRuntimeState> full_;
#endif
    AAssetManager* assets_ = nullptr;
    std::string internal_data_path_;
    verification::LaunchOptions launch_options_;
    verification::StateLimitStats verification_state_limits_;
    std::unique_ptr<verification::TimingRecorder> physics_timing_;
    std::unique_ptr<verification::TimingRecorder> render_cpu_timing_;
    std::unique_ptr<verification::TimingRecorder> render_gpu_timing_;
    std::unique_ptr<verification::GpuTimer> gpu_timer_;
    std::atomic<std::uint64_t> render_frames_{0};
    std::atomic<float> refresh_hz_{0};
    std::atomic<bool> verification_frame_available_{false};
    std::atomic<bool> verification_running_{false};
    std::atomic<bool> verification_interrupted_{false};
    std::chrono::steady_clock::time_point frame_cpu_start_{};
    bool frame_trace_open_ = false;
    bool ready_logged_ = false;
    std::atomic<bool> stop_worker_{false};
    std::thread worker_;
    std::mutex wait_mutex_;
    std::condition_variable wake_worker_;
    SnapshotMailbox mailbox_;
    JointTargetMailbox joint_targets_;
    InputMapper input_mapper_;
    ControllerSample pending_input_;
    std::string last_input_reason_;
    bool input_actions_attached_ = false;
    XrTime calibration_not_before_ = 0;
    ArmRenderer renderer_;
    PassthroughLayer passthrough_;
    bool passthrough_supported_ = false;
    int layer_capacity_ = 0;
    std::uint64_t last_render_generation_ = 0;
    std::uint64_t last_logged_generation_ = 0;
    std::size_t rendered_body_count_ = 0;
    bool underlay_logged_ = false;
    Mat4 placement_ = ProvisionalStagePlacement();
    PFN_xrCreatePassthroughFB create_passthrough_ = nullptr;
    PFN_xrDestroyPassthroughFB destroy_passthrough_ = nullptr;
    PFN_xrCreatePassthroughLayerFB create_layer_ = nullptr;
    PFN_xrDestroyPassthroughLayerFB destroy_layer_ = nullptr;
    PFN_xrRequestDisplayRefreshRateFB request_refresh_rate_ = nullptr;
    PFN_xrGetDisplayRefreshRateFB get_refresh_rate_ = nullptr;
};
}
