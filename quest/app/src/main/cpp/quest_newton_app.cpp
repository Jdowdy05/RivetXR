#include "quest_newton_app.h"
#include "android_asset.h"
#include "quest_newton/dynamic_library.h"
#include "quest_newton/newton_runtime.h"
#if defined(QUEST_NEWTON_FULL_RUNTIME)
#include "full_runtime_state.h"
#endif

#include <android/log.h>
#include <android/trace.h>
#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <type_traits>
#include <utility>

extern "C" void android_main(struct android_app* app);

namespace quest_newton {
namespace {
constexpr char kTag[] = "QuestNewton";
constexpr char kStepAsset[] = "newton/franka_step.wrp";
constexpr char kStepGraph[] = "franka_step.wrp";
constexpr char kWarpLibrary[] = "libwarp.so";
constexpr char kKernelLibrary[] = "libquest_newton_kernels_franka.so";
constexpr std::size_t kJointCount = 9;
constexpr auto kPublicationPeriod = std::chrono::milliseconds(10);

void Log(int priority, const std::string& message) {
    __android_log_print(priority, kTag, "%s", message.c_str());
}

std::string StatusText(const Status& status) {
    return std::to_string(static_cast<int>(status.code)) + ": " + status.message;
}

std::string XrResultText(const char* operation, XrResult result) {
    return std::string(operation) + " result=" + std::to_string(static_cast<int>(result));
}
} // namespace

QuestNewtonApp::QuestNewtonApp(AAssetManager* assets, const char* internal_data_path,
                               verification::LaunchOptions options)
    : assets_(assets), internal_data_path_(internal_data_path == nullptr ? "" : internal_data_path),
      launch_options_(std::move(options)),
      physics_timing_(std::make_unique<verification::TimingRecorder>(2000.0)),
      render_cpu_timing_(std::make_unique<verification::TimingRecorder>(1000000.0 / 90.0)),
      render_gpu_timing_(std::make_unique<verification::TimingRecorder>(1000000.0 / 90.0)),
      gpu_timer_(std::make_unique<verification::GpuTimer>(*render_gpu_timing_)) {
    SkipInputHandling = false;
    BackgroundColor = OVR::Vector4f(0, 0, 0, 0);
}

QuestNewtonApp::~QuestNewtonApp() { StopWorker(); }

std::vector<const char*> QuestNewtonApp::GetExtensions() {
    auto extensions = OVRFW::XrApp::GetExtensions();
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    for (const auto* name : RoomScene::Extensions()) {
        if (std::none_of(extensions.begin(), extensions.end(), [name](const char* existing) {
                return std::strcmp(existing,name)==0;
            })) extensions.push_back(name);
    }
#endif
    extensions.push_back(XR_FB_PASSTHROUGH_EXTENSION_NAME);
    if (std::none_of(extensions.begin(), extensions.end(), [](const char* name) {
            return std::strcmp(name, XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME) == 0;
        })) extensions.push_back(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);

    // v85 filters unsupported extensions instead of failing instance creation.
    // Remember availability explicitly so AppInit cannot silently fall back to VR.
    passthrough_supported_ = false;
    std::uint32_t count = 0;
    auto result = xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr);
    if (XR_FAILED(result)) {
        Fail("passthrough_extension", XrResultText("enumerate_count", result));
        return extensions;
    }
    std::vector<XrExtensionProperties> properties(count);
    for (auto& property : properties) property.type = XR_TYPE_EXTENSION_PROPERTIES;
    result = xrEnumerateInstanceExtensionProperties(nullptr, count, &count, properties.data());
    if (XR_FAILED(result)) {
        Fail("passthrough_extension", XrResultText("enumerate_properties", result));
        return extensions;
    }
    passthrough_supported_ = std::any_of(properties.begin(), properties.end(), [](const auto& property) {
        return std::strcmp(property.extensionName, XR_FB_PASSTHROUGH_EXTENSION_NAME) == 0;
    });
    return extensions;
}

bool QuestNewtonApp::ResolvePassthrough() {
    const auto resolve = [this](const char* name, auto& function) {
        PFN_xrVoidFunction address = nullptr;
        const auto result = xrGetInstanceProcAddr(Instance, name, &address);
        if (XR_FAILED(result) || address == nullptr) {
            Fail("passthrough_dispatch", XrResultText(name, result));
            return false;
        }
        function = reinterpret_cast<std::remove_reference_t<decltype(function)>>(address);
        return true;
    };
    if (!resolve("xrCreatePassthroughFB", create_passthrough_) ||
        !resolve("xrDestroyPassthroughFB", destroy_passthrough_) ||
        !resolve("xrCreatePassthroughLayerFB", create_layer_) ||
        !resolve("xrDestroyPassthroughLayerFB", destroy_layer_)) return false;

    XrSystemPassthroughProperties2FB passthrough_properties{};
    passthrough_properties.type = XR_TYPE_SYSTEM_PASSTHROUGH_PROPERTIES2_FB;
    XrSystemProperties properties{};
    properties.type = XR_TYPE_SYSTEM_PROPERTIES;
    properties.next = &passthrough_properties;
    const auto result = xrGetSystemProperties(Instance, SystemId, &properties);
    if (XR_FAILED(result)) {
        Fail("passthrough_capabilities", XrResultText("xrGetSystemProperties", result));
        return false;
    }
    if ((passthrough_properties.capabilities & XR_PASSTHROUGH_CAPABILITY_BIT_FB) == 0) {
        Fail("passthrough_capabilities", "system does not support passthrough");
        return false;
    }
    layer_capacity_ = static_cast<int>(std::min<std::uint32_t>(
        MAX_NUM_LAYERS, properties.graphicsProperties.maxLayerCount));
    if (layer_capacity_ < 2) {
        Fail("passthrough_capacity", "underlay and projection require two compositor layers");
        return false;
    }
    return true;
}

bool QuestNewtonApp::AppInit(const xrJava* context) {
    Log(ANDROID_LOG_INFO, "QUEST_NEWTON_STAGE app_init");
    if (!OVRFW::XrApp::AppInit(context)) {
        Fail("app_init", "base initialization failed");
        return false;
    }
    BackgroundColor = OVR::Vector4f(0, 0, 0, 0);
    if (!passthrough_supported_) {
        Fail("passthrough_extension", "XR_FB_passthrough is required but unavailable");
        return false;
    }
    if (!ResolvePassthrough()) return false;
    stop_worker_.store(false);
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (launch_options_.mode == verification::RunMode::live ||
        launch_options_.mode == verification::RunMode::benchmark) return InitFullRuntime(context);
    Log(ANDROID_LOG_INFO, "QUEST_LEGACY_APIC verification requested explicitly");
#else
    if (launch_options_.mode == verification::RunMode::benchmark) {
        Fail("benchmark", "Full runtime required for benchmark mode");
        return false;
    }
#endif
    worker_ = std::thread(&QuestNewtonApp::RunSmoke, this);
    Log(ANDROID_LOG_INFO, "QUEST_NEWTON_STAGE worker_started");
    return true;
}

bool QuestNewtonApp::ConfigureRefreshRate() {
    PFN_xrVoidFunction address = nullptr;
    auto result = xrGetInstanceProcAddr(Instance, "xrRequestDisplayRefreshRateFB", &address);
    if (XR_FAILED(result) || address == nullptr) {
        Fail("display_refresh", XrResultText("request dispatch", result));
        return false;
    }
    request_refresh_rate_ = reinterpret_cast<PFN_xrRequestDisplayRefreshRateFB>(address);
    address = nullptr;
    result = xrGetInstanceProcAddr(Instance, "xrGetDisplayRefreshRateFB", &address);
    if (XR_FAILED(result) || address == nullptr) {
        Fail("display_refresh", XrResultText("get dispatch", result));
        return false;
    }
    get_refresh_rate_ = reinterpret_cast<PFN_xrGetDisplayRefreshRateFB>(address);
    // Meta documents 90 Hz for Quest 3S. Request it directly; the old rate
    // enumeration API is deprecated. Record the actual rate and change events.
    result = request_refresh_rate_(Session, 90.0F);
    if (XR_FAILED(result)) {
        Fail("display_refresh", XrResultText("xrRequestDisplayRefreshRateFB", result));
        return false;
    }
    float rate = 0;
    result = get_refresh_rate_(Session, &rate);
    if (XR_FAILED(result) || !std::isfinite(rate) || rate <= 0) {
        Fail("display_refresh", XrResultText("xrGetDisplayRefreshRateFB", result));
        return false;
    }
    refresh_hz_.store(rate);
    Log(ANDROID_LOG_INFO, "QUEST_NEWTON_REFRESH_HZ value=" + std::to_string(rate));
    return true;
}

bool QuestNewtonApp::SessionInit() {
    // v85 MainLoop ignores the return value of InitSession. Also request exit
    // on failure so it cannot continue submitting an incomplete composition.
    const auto reject = [this](const char* stage, const std::string& detail) {
        Fail(stage, detail);
        ShouldExit = true;
        return false;
    };
    if (!OVRFW::XrApp::SessionInit()) return reject("session_init", "base initialization failed");
    if (!ConfigureRefreshRate()) return reject("session_init", "90 Hz refresh request failed");
    if (!input_actions_attached_ || RightControllerGripSpace == XR_NULL_HANDLE) {
        return reject("input_session", "Touch actions or grip space are unavailable");
    }
    if (StageSpace != XR_NULL_HANDLE) {
        CurrentSpace = StageSpace;
        Log(ANDROID_LOG_INFO, "QUEST_NEWTON_SPACE STAGE placement=provisional base_m=0,0,-1.2");
    } else if (LocalSpace != XR_NULL_HANDLE) {
        CurrentSpace = LocalSpace;
        Log(ANDROID_LOG_WARN, "QUEST_NEWTON_SPACE LOCAL visualization_fallback=uncalibrated base_m=0,0,-1.2");
    } else {
        return reject("reference_space", "neither STAGE nor LOCAL space is available");
    }

    PassthroughDispatch dispatch;
    dispatch.create_feature = [this](std::uint64_t& handle) {
        XrPassthroughCreateInfoFB info{};
        info.type = XR_TYPE_PASSTHROUGH_CREATE_INFO_FB;
        info.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
        XrPassthroughFB feature = XR_NULL_HANDLE;
        const auto result = create_passthrough_(Session, &info, &feature);
        handle = reinterpret_cast<std::uint64_t>(feature);
        if (XR_FAILED(result)) Fail("passthrough_create", XrResultText("xrCreatePassthroughFB", result));
        return XR_SUCCEEDED(result) && feature != XR_NULL_HANDLE;
    };
    dispatch.create_layer = [this](std::uint64_t feature, std::uint64_t& handle) {
        XrPassthroughLayerCreateInfoFB info{};
        info.type = XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB;
        info.passthrough = reinterpret_cast<XrPassthroughFB>(feature);
        info.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
        info.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
        XrPassthroughLayerFB layer = XR_NULL_HANDLE;
        const auto result = create_layer_(Session, &info, &layer);
        handle = reinterpret_cast<std::uint64_t>(layer);
        if (XR_FAILED(result)) Fail("passthrough_layer_create", XrResultText("xrCreatePassthroughLayerFB", result));
        return XR_SUCCEEDED(result) && layer != XR_NULL_HANDLE;
    };
    dispatch.destroy_layer = [this](std::uint64_t handle) {
        const auto result = destroy_layer_(reinterpret_cast<XrPassthroughLayerFB>(handle));
        if (XR_FAILED(result)) Fail("passthrough_layer_destroy", XrResultText("xrDestroyPassthroughLayerFB", result));
        return XR_SUCCEEDED(result);
    };
    dispatch.destroy_feature = [this](std::uint64_t handle) {
        const auto result = destroy_passthrough_(reinterpret_cast<XrPassthroughFB>(handle));
        if (XR_FAILED(result)) Fail("passthrough_destroy", XrResultText("xrDestroyPassthroughFB", result));
        return XR_SUCCEEDED(result);
    };
    if (!passthrough_.Start(std::move(dispatch))) return reject("passthrough_start", passthrough_.Error());
    if (!renderer_.Init(assets_) || glGetError() != GL_NO_ERROR) {
        passthrough_.Stop([this] { renderer_.Shutdown(); });
        return reject("renderer_init", "Franka visual geometry or shader initialization failed");
    }
    if (!gpu_timer_->Init()) {
        passthrough_.Stop([this] { renderer_.Shutdown(); });
        return reject("gpu_timer_init", "GPU timing resource initialization failed");
    }
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_ && !full_->environment.Init()) return reject("environment_init", "grid/box renderer initialization failed");
    if (full_) {
        if(!full_->menu.SessionInit())return reject("menu_pointer_init","controller pointer initialization failed");
        InitRemoteSceneSession();
        full_->room_api_available=full_->room_scene.Init(Instance,Session);
        full_->room_scene.SetEnabled(full_->settings.environment_collisions);
        full_->room_refresh_pending=full_->settings.environment_collisions;
    }
#endif
    last_render_generation_ = 0;
    last_logged_generation_ = 0;
    underlay_logged_ = false;
    Log(ANDROID_LOG_INFO, "QUEST_NEWTON_PASSTHROUGH_OK reconstruction=running clear_rgba=0,0,0,0");
    return true;
}

void QuestNewtonApp::SessionEnd() {
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_) {
        SuspendFullInput(true);
        StopRemoteSceneSession(GetContext());
        full_->menu.SessionEnd();
        full_->room_scene.Shutdown();
        full_->room_api_available=false;
        full_->environment.Shutdown();
    }
#endif
    verification_frame_available_.store(false);
    gpu_timer_->Shutdown();
    input_mapper_.InvalidateCalibration();
    joint_targets_.Publish(input_mapper_.Current().joints, false);
    input_actions_attached_ = false;
    // XrApp invokes this before destroying Session and the current EGL context.
    // The host-tested owner releases GL, then the layer, then the feature.
    if (!passthrough_.Stop([this] { renderer_.Shutdown(); })) {
        Fail("session_end", passthrough_.Error());
    }
    OVRFW::XrApp::SessionEnd();
}

void QuestNewtonApp::AppShutdown(const xrJava* context) {
    StopWorker();
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    ShutdownFullRuntime(context);
#endif
    OVRFW::XrApp::AppShutdown(context);
}

bool QuestNewtonApp::CheckInputResult(const char* operation, XrResult result) {
    if (XR_SUCCEEDED(result)) return true;
    Fail("input_openxr", XrResultText(operation, result));
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_) return false;
#endif
    ShouldExit = true;
    return false;
}

void QuestNewtonApp::AttachActionSets() {
    input_actions_attached_ = false;
    if (BaseActionSet == XR_NULL_HANDLE || GripPoseAction == XR_NULL_HANDLE ||
        IndexTriggerAction == XR_NULL_HANDLE || ButtonAAction == XR_NULL_HANDLE ||
        RightHandPath == XR_NULL_PATH || ActionSets.empty()) {
        Fail("input_actions", "required Touch action handles are missing");
        ShouldExit = true;
        return;
    }
    XrSessionActionSetsAttachInfo info{};
    info.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
    info.countActionSets = static_cast<std::uint32_t>(ActionSets.size());
    info.actionSets = ActionSets.data();
    input_actions_attached_ = CheckInputResult("xrAttachSessionActionSets", xrAttachSessionActionSets(Session, &info));
}

void QuestNewtonApp::SyncActionSets(OVRFW::ovrApplFrameIn& in) {
    frame_cpu_start_ = std::chrono::steady_clock::now();
    ATrace_beginSection("QuestNewton.FrameCPU");
    frame_trace_open_ = true;
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_) { SyncFullInput(in); return; }
#endif
    // The SDK creates/suggests the Touch actions. Sync exactly once and query
    // the full active/valid/tracked contract instead of its reduced frame flags.
    pending_input_ = {};
    if (launch_options_.mode != verification::RunMode::live) return;
    const XrTime sample_time = ToXrTime(in.PredictedDisplayTime);
    pending_input_.stage_valid = StageSpace != XR_NULL_HANDLE && CurrentSpace == StageSpace &&
        sample_time >= calibration_not_before_;
    if (!input_actions_attached_) return;
    XrActiveActionSet active{BaseActionSet, XR_NULL_PATH};
    XrActionsSyncInfo sync{};
    sync.type = XR_TYPE_ACTIONS_SYNC_INFO;
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    const auto sync_result = xrSyncActions(Session, &sync);
    if (!CheckInputResult("xrSyncActions", sync_result)) return;
    pending_input_.focused = Focused && sync_result == XR_SUCCESS;
    if (!pending_input_.focused) return;

    XrActionStateGetInfo get{};
    get.type = XR_TYPE_ACTION_STATE_GET_INFO;
    get.subactionPath = RightHandPath;
    get.action = GripPoseAction;
    XrActionStatePose pose{};
    pose.type = XR_TYPE_ACTION_STATE_POSE;
    if (!CheckInputResult("xrGetActionStatePose", xrGetActionStatePose(Session, &get, &pose))) return;
    pending_input_.pose_active = pose.isActive == XR_TRUE;
    get.action = IndexTriggerAction;
    XrActionStateFloat trigger{};
    trigger.type = XR_TYPE_ACTION_STATE_FLOAT;
    if (!CheckInputResult("xrGetActionStateFloat", xrGetActionStateFloat(Session, &get, &trigger))) return;
    pending_input_.trigger_active = trigger.isActive == XR_TRUE;
    pending_input_.trigger = trigger.currentState;
    get.action = ButtonAAction;
    XrActionStateBoolean calibrate{};
    calibrate.type = XR_TYPE_ACTION_STATE_BOOLEAN;
    if (!CheckInputResult("xrGetActionStateBoolean", xrGetActionStateBoolean(Session, &get, &calibrate))) return;
    pending_input_.calibrate_active = calibrate.isActive == XR_TRUE;
    pending_input_.calibrate_pressed = calibrate.currentState == XR_TRUE;
    if (!pending_input_.pose_active || !pending_input_.stage_valid) return;
    XrSpaceLocation location{};
    location.type = XR_TYPE_SPACE_LOCATION;
    if (!CheckInputResult("xrLocateSpace", xrLocateSpace(RightControllerGripSpace, StageSpace, sample_time, &location))) return;
    const auto flags = location.locationFlags;
    pending_input_.position_valid = (flags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
    pending_input_.orientation_valid = (flags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
    pending_input_.position_tracked = (flags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
    pending_input_.orientation_tracked = (flags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) != 0;
    const auto& p = location.pose.position;
    const auto& q = location.pose.orientation;
    pending_input_.stage_from_grip = {{p.x, p.y, p.z}, {q.x, q.y, q.z, q.w}};
}

void QuestNewtonApp::PublishInput(const ControllerSample& sample) {
    ATrace_beginSection("QuestNewton.IK");
    const auto& mapped = input_mapper_.Update(sample);
    ATrace_endSection();
    if (!joint_targets_.Publish(mapped.joints, mapped.engaged)) {
        Fail("target_publish", "joint targets violate the finite URDF limit contract");
        ShouldExit = true;
        return;
    }
    if (mapped.calibrated) {
        const auto stage_from_base = kinematics::Inverse(mapped.robot_base_from_stage);
        const auto placement = PoseMatrix({stage_from_base.position, stage_from_base.rotation});
        if (placement_ != placement) {
            placement_ = placement;
            last_render_generation_ = 0;
        }
    }
    if (mapped.reason != last_input_reason_) {
        last_input_reason_ = mapped.reason;
        Log(ANDROID_LOG_INFO, "QUEST_NEWTON_INPUT state=" + last_input_reason_ +
            " calibrated=" + std::to_string(mapped.calibrated) +
            " engaged=" + std::to_string(mapped.engaged));
    }
}

void QuestNewtonApp::Update(const OVRFW::ovrApplFrameIn& in) {
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_) { UpdateFullRuntime(in); return; }
#else
    (void)in;
#endif
    if (launch_options_.mode == verification::RunMode::live) PublishInput(pending_input_);
}

void QuestNewtonApp::SessionStateChanged(XrSessionState state) {
    if (state != XR_SESSION_STATE_FOCUSED) {
        verification_frame_available_.store(false);
        if (verification_running_.load()) verification_interrupted_.store(true);
#if defined(QUEST_NEWTON_FULL_RUNTIME)
        if (full_) SuspendFullInput(false);
        else
#endif
        if (launch_options_.mode == verification::RunMode::live) PublishInput({});
    }
    OVRFW::XrApp::SessionStateChanged(state);
}

void QuestNewtonApp::AppHandleEvent(XrEventDataBaseHeader* event) {
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_ && event) full_->room_scene.OnEvent(event);
#endif
    if (event != nullptr && event->type == XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB) {
        const auto* change = reinterpret_cast<const XrEventDataDisplayRefreshRateChangedFB*>(event);
        if (std::isfinite(change->toDisplayRefreshRate) && change->toDisplayRefreshRate > 0) {
            refresh_hz_.store(change->toDisplayRefreshRate);
            if (verification_running_.load() && std::abs(change->toDisplayRefreshRate - 90.0F) >= .1F) {
                verification_interrupted_.store(true);
            }
            Log(ANDROID_LOG_INFO, "QUEST_NEWTON_REFRESH_HZ value=" + std::to_string(change->toDisplayRefreshRate));
        }
    }
    if (event != nullptr && event->type == XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING) {
        const auto* change = reinterpret_cast<const XrEventDataReferenceSpaceChangePending*>(event);
        if (change->session == Session && change->referenceSpaceType == XR_REFERENCE_SPACE_TYPE_STAGE) {
            calibration_not_before_ = change->changeTime;
#if defined(QUEST_NEWTON_FULL_RUNTIME)
            if (full_) SuspendFullInput(true);
            else {
#endif
            input_mapper_.InvalidateCalibration();
            if (launch_options_.mode == verification::RunMode::live) PublishInput({});
            else {
                verification_frame_available_.store(false);
                if (verification_running_.load()) verification_interrupted_.store(true);
            }
#if defined(QUEST_NEWTON_FULL_RUNTIME)
            }
#endif
            Log(ANDROID_LOG_INFO, "QUEST_NEWTON_INPUT calibration_invalidated=stage_changed");
        }
    }
    OVRFW::XrApp::AppHandleEvent(event);
}

void QuestNewtonApp::PreProjectionAddLayer(xrCompositorLayerUnion* layers, int& layer_count) {
    const int index = passthrough_.ReserveUnderlay(layer_capacity_, layer_count);
    if (layers == nullptr || index < 0) {
        Fail("passthrough_submit", "underlay unavailable or no capacity remains for projection");
        ShouldExit = true;
        return;
    }
    XrCompositionLayerPassthroughFB layer{};
    layer.type = XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB;
    layer.flags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
    layer.space = XR_NULL_HANDLE;
    layer.layerHandle = reinterpret_cast<XrPassthroughLayerFB>(passthrough_.Handle());
    layers[index].Passthrough = layer;
    if (!underlay_logged_) {
        Log(ANDROID_LOG_INFO, "QUEST_NEWTON_COMPOSITION underlay=" + std::to_string(index) +
            " next_projection=" + std::to_string(layer_count) + " blend=opaque");
        underlay_logged_ = true;
    }
}

void QuestNewtonApp::Render(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out) {
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_) { RenderFullRuntime(in, out); return; }
#else
    (void)in;
#endif
    // Render is invoked only when the runtime says shouldRender. Let recorded
    // physics begin on a real focused frame; its first publication fills the
    // initially empty scene, avoiding an initialization dependency cycle.
    if (Focused && std::abs(refresh_hz_.load() - 90.0F) < .1F) {
        verification_frame_available_.store(true);
        wake_worker_.notify_all();
    }
    // Read releases the short mailbox mutex before any transform or GL work.
    const auto snapshot = mailbox_.Read();
    if (snapshot.placement_valid) placement_ = snapshot.stage_from_base;
    if (snapshot.valid && snapshot.generation != last_render_generation_) {
        if (!renderer_.Update(snapshot, placement_)) {
            Fail("renderer_update", "complete body snapshot was rejected; previous pose preserved");
            ShouldExit = true;
            return;
        }
        last_render_generation_ = snapshot.generation;
    }
    const auto initial_count = out.Surfaces.size();
    // Use the located head pose directly; controller target mapping never
    // consumes it and remains fixed in calibrated STAGE space.
    renderer_.Append(out.FrameMatrices.CenterView, out.Surfaces);
    rendered_body_count_ = out.Surfaces.size() - initial_count;
    if (last_render_generation_ != 0 && rendered_body_count_ != renderer_.VisualCount()) {
        Fail("renderer_append", "expected every nonempty Franka visual surface");
        ShouldExit = true;
    }
}

void QuestNewtonApp::AppRenderFrame(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out) {
    rendered_body_count_ = 0;
    auto* frame_gpu_timer=gpu_timer_.get();
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if(full_){
        full_->remote_frame_rendered=false;
        if(full_->remote_inspecting)frame_gpu_timer=full_->remote_timer_ready?&full_->remote_gpu_timer:nullptr;
    }
#endif
    if(frame_gpu_timer)frame_gpu_timer->Begin();
    OVRFW::XrApp::AppRenderFrame(in, out);
    const bool rendered = renderer_.Ready() && rendered_body_count_ == renderer_.VisualCount();
    bool record_frame=rendered;
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    const auto scope=RenderTimingScope(rendered,full_ && full_->remote_frame_rendered);
    record_frame=scope!=InspectionTimingScope::None;
    if(scope==InspectionTimingScope::Remote){
        const double elapsed_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-frame_cpu_start_).count();
        full_->remote_cpu_timing.Observe(elapsed_us);++full_->remote_render_frames;
    }
#endif
    if(frame_gpu_timer)frame_gpu_timer->End(record_frame);
    if (frame_trace_open_) {
        ATrace_endSection();
        frame_trace_open_ = false;
    }
    if (rendered) {
        const double elapsed_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - frame_cpu_start_).count();
        render_cpu_timing_->Observe(elapsed_us);
        render_frames_.fetch_add(1);
#if defined(QUEST_NEWTON_FULL_RUNTIME)
        if (full_ && !ready_logged_) {
            const auto snapshot = full_->scenes.Read();
            bool fault;
            { const std::lock_guard lock(full_->mutex); fault = full_->fault; }
            if (!fault && snapshot.step_index > 0 && rendered_body_count_ == renderer_.VisualCount() &&
                std::abs(refresh_hz_.load() - 90.0F) < .1F) {
                Log(ANDROID_LOG_INFO, "QUEST_FULL_READY refresh_hz=" + std::to_string(refresh_hz_.load()) +
                    " visuals=" + std::to_string(rendered_body_count_) + " step=" + std::to_string(snapshot.step_index));
                ready_logged_ = true;
            }
        }
        if (!full_)
#endif
        if (!ready_logged_ && std::abs(refresh_hz_.load() - 90.0F) < .1F) {
            Log(ANDROID_LOG_INFO, "QUEST_NEWTON_READY mode=" + std::string(verification::ModeName(launch_options_.mode)) +
                " run_id=" + launch_options_.run_id + " refresh_hz=90 visuals=" + std::to_string(rendered_body_count_));
            ready_logged_ = true;
        }
    }
    const auto error = glGetError();
    if (error != GL_NO_ERROR) {
        Fail("renderer_frame", "OpenGL error=" + std::to_string(error));
        ShouldExit = true;
        return;
    }
    // Emit evidence after the framework has drawn both eyes, only on actual
    // render frames, with a bounded cadence while the physics worker advances.
    if (
#if defined(QUEST_NEWTON_FULL_RUNTIME)
        !full_ &&
#endif
        renderer_.Ready() && rendered_body_count_ == renderer_.VisualCount() &&
        (last_logged_generation_ == 0 || last_render_generation_ - last_logged_generation_ >= 100)) {
        Log(ANDROID_LOG_INFO, "QUEST_NEWTON_OVERLAY_OK generation=" + std::to_string(last_render_generation_) +
            " bodies=" + std::to_string(kArmBodies) + " visuals=" + std::to_string(rendered_body_count_));
        last_logged_generation_ = last_render_generation_;
    }
}

void QuestNewtonApp::Fail(const char* stage, const std::string& detail) {
#if defined(QUEST_NEWTON_FULL_RUNTIME)
    if (full_) {
        Log(ANDROID_LOG_ERROR, std::string("QUEST_FULL_FAULT stage=") + stage + " detail=" + detail);
        const std::lock_guard lock(full_->mutex);
        full_->status.diagnostic = std::string(stage) + ": " + detail;
        full_->fault = true;
        return;
    }
#endif
    Log(ANDROID_LOG_ERROR, std::string("QUEST_NEWTON_SMOKE_FAIL stage=") + stage + " detail=" + detail);
    if (launch_options_.mode != verification::RunMode::live) {
        Log(ANDROID_LOG_ERROR, "QUEST_NEWTON_VERIFY_FAIL run_id=" + launch_options_.run_id +
            " stage=" + stage + " detail=" + detail);
    }
}

bool QuestNewtonApp::TimedStep(NewtonRuntime& simulation, bool check_limits) {
    const auto start = std::chrono::steady_clock::now();
    ATrace_beginSection("QuestNewton.PhysicsStep");
    const auto status = simulation.Step(1);
    std::string state_error;
    const bool valid_state = !status.IsOk() || !check_limits || ValidateVerificationState(simulation, state_error);
    ATrace_endSection();
    const double elapsed_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count();
    if (!physics_timing_->Observe(elapsed_us)) {
        Fail("physics_timing", "invalid or exhausted timing accumulator");
        return false;
    }
    if (!status.IsOk()) {
        Fail("step", StatusText(status));
        return false;
    }
    if (!valid_state) {
        Fail("verification_state", state_error);
        return false;
    }
    return true;
}

void QuestNewtonApp::StopWorker() {
    {
        const std::lock_guard lock(wait_mutex_);
        stop_worker_.store(true);
    }
    wake_worker_.notify_all();
    if (worker_.joinable()) worker_.join();
}

bool QuestNewtonApp::NativeLibraryDirectory(std::filesystem::path& directory) {
    Dl_info info{};
    if (dladdr(reinterpret_cast<void*>(&android_main), &info) == 0 || info.dli_fname == nullptr) {
        Fail("native_path", "dladdr failed");
        return false;
    }
    directory = std::filesystem::path(info.dli_fname).parent_path();
    if (directory.empty()) {
        Fail("native_path", "native library directory is empty");
        return false;
    }
    return true;
}

void QuestNewtonApp::RunSmoke() {
    try {
        Log(ANDROID_LOG_INFO, "QUEST_NEWTON_STAGE worker_enter");
        if (assets_ == nullptr || internal_data_path_.empty()) {
            Fail("startup", "Android asset manager or internal path is unavailable");
            return;
        }
        std::string extraction_error;
        if (!ExtractAssetAtomically(assets_, kStepAsset, internal_data_path_.c_str(),
                                    kStepGraph, &extraction_error)) {
            Fail("asset_extract", extraction_error);
            return;
        }
        std::filesystem::path native_directory;
        if (!NativeLibraryDirectory(native_directory)) return;

        auto warp = NativeDynamicLibrary::Open(
            (native_directory / kWarpLibrary).string(), NativeDynamicLibrary::Visibility::global);
        if (!warp.IsOk()) {
            Fail("warp_load", warp.status().message);
            return;
        }
        auto kernels = NativeDynamicLibrary::Open(
            (native_directory / kKernelLibrary).string(), NativeDynamicLibrary::Visibility::local);
        if (!kernels.IsOk()) {
            Fail("kernel_load", kernels.status().message);
            return;
        }

        RuntimeConfig config;
        config.expected_warp_version = "1.18.0.dev2";
        config.graph_path = (std::filesystem::path(internal_data_path_) / kStepGraph).string();
        config.joint_count = kJointCount;
        config.body_count = kArmBodies;
        for (const auto& buffer : std::array<std::pair<const char*, std::size_t>, 8>{{
                 {"joint_q_in", 36}, {"joint_qd_in", 36}, {"joint_force", 36},
                 {"joint_target_q", 36}, {"joint_target_qd", 36}, {"joint_q_out", 36},
                 {"joint_qd_out", 36}, {"body_q_out", 336}}}) {
            config.required_buffers.push_back({buffer.first, buffer.second});
        }
        config.warp_library = std::move(warp).value();
        config.kernel_libraries.push_back(std::move(kernels).value());
        auto runtime = NewtonRuntime::Create(std::move(config));
        if (!runtime.IsOk()) {
            Fail("runtime_create", StatusText(runtime.status()));
            return;
        }
        constexpr std::array<float, kJointCount> home = {0.0F, -0.569F, 0.0F, -2.81F, 0.0F,
                                                       3.037F, 0.741F, 0.04F, 0.04F};
        constexpr std::array<float, kJointCount> zero = {};
        auto& simulation = runtime.value();
        const auto reset = simulation->Reset(home, zero);
        if (!reset.IsOk()) {
            Fail("reset", StatusText(reset));
            return;
        }
        if (launch_options_.mode != verification::RunMode::live) {
            RunVerification(*simulation);
            return;
        }
        const float first = simulation->JointPositions().front();
        bool smoke_reported = false;
        std::uint64_t consumed_target_generation = 0;
        std::uint64_t logged_target_generation = 0;
        bool worker_engaged = false;
        while (!stop_worker_.load()) {
            const auto cycle_start = std::chrono::steady_clock::now();
            const auto command = joint_targets_.Read();
            if (command.engaged != worker_engaged) {
                worker_engaged = command.engaged;
                Log(ANDROID_LOG_INFO, "QUEST_NEWTON_WORKER engaged=" + std::to_string(worker_engaged));
            }
            if (command.generation != consumed_target_generation) {
                if (command.engaged) {
                    auto targets = home; // The two independent finger targets stay at 0.04 m.
                    std::copy(command.joints.begin(), command.joints.end(), targets.begin());
                    const auto status = simulation->SetJointTargets(targets, zero);
                    if (!status.IsOk()) {
                        Fail("targets", StatusText(status));
                        return;
                    }
                    consumed_target_generation = command.generation;
                    if (logged_target_generation == 0 || command.generation - logged_target_generation >= 100) {
                        Log(ANDROID_LOG_INFO, "QUEST_NEWTON_TARGET_APPLIED generation=" + std::to_string(command.generation) +
                            " q0=" + std::to_string(targets[0]) + " q3=" + std::to_string(targets[3]));
                        logged_target_generation = command.generation;
                    }
                }
            }
            for (int substep = 0; substep < 10; ++substep) if (!TimedStep(*simulation)) return;
            // Publish copies all 84 values before the worker can replay again.
            // A rejected/faulted generation never overwrites the last valid one.
            if (!mailbox_.Publish(simulation->BodyTransforms())) {
                Fail("snapshot_publish", "body transform snapshot is invalid");
                return;
            }
            const float last = simulation->JointPositions().front();
            if (!std::isfinite(first) || !std::isfinite(last)) {
                Fail("finite", "reset or step output was non-finite");
                return;
            }
            if (!smoke_reported) {
                Log(ANDROID_LOG_INFO, "QUEST_NEWTON_SMOKE_OK first=" + std::to_string(first) +
                    " last=" + std::to_string(last));
                smoke_reported = true;
            }
            // No catch-up bursts: ten 1 ms substeps per cycle, with at least a
            // short interruptible yield even if replay overruns its 10 ms slot.
            const auto wake_at = std::max(cycle_start + kPublicationPeriod,
                std::chrono::steady_clock::now() + std::chrono::milliseconds(1));
            std::unique_lock lock(wait_mutex_);
            wake_worker_.wait_until(lock, wake_at, [this] { return stop_worker_.load(); });
        }
    } catch (const std::exception& exception) {
        Fail("exception", exception.what());
    } catch (...) {
        Fail("exception", "unknown exception");
    }
}
} // namespace quest_newton
