#include "quest_newton_app.h"
#include "full_runtime_state.h"
#include "python_session.h"
#include "registered_input.h"
#include "simulation_clock.h"
#include "full_benchmark.h"
#include "full_control.h"
#include <android/log.h>
#include <android/trace.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <locale>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace quest_newton {
namespace {
namespace kin = kinematics;
double WallNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void FullLog(int priority, const std::string& text) {
    __android_log_print(priority, "QuestNewton", "%s", text.c_str());
}
kin::Pose FromXr(const XrPosef& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
}
OVR::Posef ToOvr(const kin::Pose& pose) {
    return {OVR::Quatf(pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]),
            OVR::Vector3f(pose.position[0], pose.position[1], pose.position[2])};
}
bool Tracked(XrSpaceLocationFlags flags) {
    constexpr auto required = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
        XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    return (flags & required) == required;
}
bool SamePose(const kin::Pose& a, const kin::Pose& b) {
    return a.position == b.position && a.rotation == b.rotation;
}
std::string InitialSettings(const SimSettings& settings, const kin::Pose& pose) {
    auto json = SettingsJson(settings);
    json.pop_back();
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << json << ",\"initial_base_pose\":[" << std::setprecision(9);
    for (std::size_t i=0; i<3; ++i) text << (i ? "," : "") << pose.position[i];
    for (const auto value : pose.rotation) text << ',' << value;
    text << "]}";
    return text.str();
}
SceneSnapshot Decode(const std::vector<std::byte>& bytes) {
    SceneSnapshot snapshot;
    std::string error;
    if (!DecodeSceneSnapshot(bytes, snapshot, error)) throw std::runtime_error("scene snapshot: " + error);
    if (snapshot.bodies.size() < kArmBodies) throw std::runtime_error("scene is missing Franka bodies");
    return snapshot;
}
bool ValidInput(const FullInputFrame& input) {
    return ValidFullInput(input,std::chrono::steady_clock::now());
}
std::string JsonString(const std::string& value) {
    std::ostringstream out;out << '"';
    for (const unsigned char c : value) {
        if(c=='"' || c=='\\')out << '\\' << static_cast<char>(c);
        else if(c<32)out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
        else out << static_cast<char>(c);
    }
    out << '"';return out.str();
}
void ResolveRoomPermissionMethods(FullRuntimeState& state,JNIEnv* env) {
    state.room_permission_query=env->GetStaticMethodID(state.bridge,"hasScenePermission","(Landroid/content/Context;)Z");
    if (env->ExceptionCheck()) {env->ExceptionClear();state.room_permission_query=nullptr;}
    state.room_permission_request=env->GetStaticMethodID(state.bridge,"requestScenePermission","(Landroid/app/Activity;)V");
    if (env->ExceptionCheck()) {env->ExceptionClear();state.room_permission_request=nullptr;}
}
} // namespace

bool QuestNewtonApp::InitFullRuntime(const xrJava* context) {
    full_ = std::make_unique<FullRuntimeState>();
    std::string load_error;
    const auto settings_path = std::filesystem::path(internal_data_path_) / "simulation-settings-v1.txt";
    const bool benchmark = launch_options_.mode == verification::RunMode::benchmark;
    if (!benchmark && std::filesystem::exists(settings_path) && !LoadSettings(settings_path, full_->settings, load_error)) {
        full_->settings = SimSettings{};
        full_->status.diagnostic = "Saved settings invalid; using defaults: " + load_error;
    }
    if (benchmark) full_->settings = BenchmarkSettings(launch_options_);
    full_->requested_settings = full_->settings;
    SimMenuCallbacks callbacks;
    callbacks.applySettings = [this, settings_path](const SimSettings& settings, std::string& error) {
        if (!ValidateSettings(settings, error) || !SaveSettings(settings_path, settings, error)) return false;
        if (settings.environment_collisions != full_->settings.environment_collisions) {
            full_->room_permission_requested=false;
            full_->room_permission_poll=-1;
        }
        if (settings.bindings != full_->settings.bindings) {
            // The button used to edit a binding may still be held. Observe its
            // release before treating it as a new menu/calibration press.
            full_->menu_down = true;
            full_->calibration_down = true;
        }
        full_->settings = settings;
        {
            const std::lock_guard lock(full_->mutex);
            full_->requested_settings = settings;
            ++full_->settings_generation;
            full_->status.settings_pending = true;
            if (!full_->fault) full_->status.diagnostic = "Settings saved; waiting for simulation worker";
        }
        // Persisted values reach the worker at its next boundary; render-only
        // choices can be used immediately without waiting for Python.
        wake_worker_.notify_all();
        return true;
    };
    callbacks.command = [this](SimMenuCommand command) {
        auto& f=*full_;
        if(HandleRemoteCommand(command))return;
        if(command==SimMenuCommand::ToggleDiagnostics){
            const std::lock_guard lock(f.mutex);f.show_diagnostics=!f.show_diagnostics;
            f.status.show_diagnostics=f.show_diagnostics;return;
        }
        if(command==SimMenuCommand::RecordReset){
            const std::lock_guard lock(f.mutex);
            if(f.fault || !f.scenes.Read().publication || f.status.settings_pending){
                f.status.diagnostic="Wait for a configured running scene before recording";return;
            }
        }
        if(command==SimMenuCommand::PlaceCube || command==SimMenuCommand::SelectNextCube ||
           command==SimMenuCommand::MoveCube || command==SimMenuCommand::ResetCube || command==SimMenuCommand::DeleteCube){
            ObjectEdit edit;bool begin=false;
            {
                const std::lock_guard lock(f.mutex);
                const auto scene=f.scenes.Read();
                if(f.fault || !scene.publication || f.details.model_generation!=scene.model_generation){
                    f.status.diagnostic="Wait for a valid scene and object list";return;
                }
                std::vector<DetailedObject> objects;
                for(const auto& object:f.details.objects)if(object.id)objects.push_back(object);
                auto selected=std::find_if(objects.begin(),objects.end(),[&](const auto& object){return object.id==f.selected_object;});
                if(command==SimMenuCommand::SelectNextCube){
                    if(objects.empty())f.selected_object=0;
                    else f.selected_object=(selected==objects.end() || ++selected==objects.end())?objects.front().id:selected->id;
                    return;
                }
                if(command!=SimMenuCommand::PlaceCube && selected==objects.end()){
                    f.status.diagnostic="Select a user cube first";return;
                }
                if(command==SimMenuCommand::PlaceCube && (objects.size()>=8 || f.next_object_id==std::numeric_limits<std::uint32_t>::max())){
                    f.status.diagnostic="Cube capacity reached; delete a cube first";return;
                }
                if(RoomPhysicsBlocked(f.settings.environment_collisions,f.room_ready,
                    RoomFrameCurrent(f.input,std::chrono::steady_clock::now()),*f.desired_room,scene.room.get())){
                    f.status.diagnostic="Wait for room alignment before editing cubes";return;
                }
                edit.kind=command==SimMenuCommand::PlaceCube?ObjectEditKind::Spawn:command==SimMenuCommand::MoveCube?ObjectEditKind::Move:
                    command==SimMenuCommand::ResetCube?ObjectEditKind::Reset:ObjectEditKind::Remove;
                edit.id=edit.kind==ObjectEditKind::Spawn?f.next_object_id++:selected->id;
                if(selected!=objects.end())edit.half_extents=selected->half_extents;
                edit.reference=f.reference_counter;edit.settings_generation=f.settings_generation;edit.suspension=f.suspension_counter;
                edit.room_revision=scene.room?scene.room->revision:0;
                begin=edit.kind==ObjectEditKind::Spawn || edit.kind==ObjectEditKind::Move;
                if(!begin){
                    if(f.object_edits.size()>=16){f.status.diagnostic="Object queue busy; try again";return;}
                    f.object_edits.push_back(edit);f.status.diagnostic="Object edit queued";
                }
            }
            if(begin){
                f.menu.SetVisible(false,ToOvr(f.head));SuspendFullInput(false);
                edit.suspension=f.suspension_counter;f.object_placement.Begin(edit);
                const std::lock_guard lock(f.mutex);
                f.status.diagnostic="Aim at floor or table; release then press right index. Menu cancels.";
            }
            wake_worker_.notify_all();return;
        }
        if (command==SimMenuCommand::RefreshRoom || command==SimMenuCommand::ScanRoom) {
            if (!full_->settings.environment_collisions) {
                const std::lock_guard lock(full_->mutex);
                if(!full_->fault)full_->status.diagnostic="Turn on environment collisions before loading or scanning a room";
                return;
            }
            full_->room_permission_requested=false;
            full_->room_permission_poll=-1;
            bool accepted=true;
            if (command==SimMenuCommand::ScanRoom) accepted=full_->room_scene.RequestCapture();
            else full_->room_scene.Refresh();
            {
                const std::lock_guard lock(full_->mutex);
                if(!full_->fault)full_->status.diagnostic=accepted?std::string{}:full_->room_scene.Status();
            }
            wake_worker_.notify_all();
            return;
        }
        if (command == SimMenuCommand::Recenter) {
            SuspendFullInput(true);
            return;
        }
        if (command == SimMenuCommand::RestoreDefaults) {
            // TinyUI already invokes applySettings(defaults) transactionally.
            return;
        } else {
            if (command == SimMenuCommand::Reset && (!full_->bridge || !full_->activity ||
                !full_->room_permission_query || !full_->room_permission_request)) {
                try {
                    const auto* context = GetContext();
                    auto activity = full_->activity;
                    if (!activity) activity = context->Env->NewGlobalRef(context->ActivityObject);
                    auto bridge = full_->bridge;
                    if (!bridge) bridge = LoadSimulationBridge(context->Env, context->ActivityObject);
                    const std::lock_guard lock(full_->mutex);
                    full_->activity = activity;
                    full_->bridge = bridge;
                    ResolveRoomPermissionMethods(*full_,context->Env);
                } catch (const std::exception& error) { Fail("java_bridge_retry", error.what()); return; }
            }
            const std::lock_guard lock(full_->mutex);
            if (full_->commands.size() >= 16) {
                full_->status.diagnostic = "Simulation command queue busy; try again";
                return;
            }
            full_->commands.push_back(command);
        }
        wake_worker_.notify_all();
    };
    if (!full_->menu.Init(context, GetFileSys(), full_->settings, std::move(callbacks))) {
        Fail("menu_init", "TinyUI initialization failed");
        return false;
    }
    // Activity and bridge references outlive the worker and are deleted only
    // after it has detached. Class lookup uses the Activity class loader.
    try {
        full_->vm = context->Vm;
        full_->activity = context->Env->NewGlobalRef(context->ActivityObject);
        if (!full_->activity) throw std::runtime_error("Activity global reference failed");
        full_->bridge = LoadSimulationBridge(context->Env, context->ActivityObject);
        ResolveRoomPermissionMethods(*full_,context->Env);
    } catch (const std::exception& error) {
        Fail("java_bridge", error.what());
        // Retry resolution on Reset using the render thread's valid loader.
    }
    InitRemoteScene(context);
    worker_ = std::thread(&QuestNewtonApp::RunFullSimulation, this);
    FullLog(ANDROID_LOG_INFO, "QUEST_FULL_STAGE worker_started waiting_for_tracked_STAGE");
    return true;
}

void QuestNewtonApp::ShutdownFullRuntime(const xrJava* context) {
    if (!full_) return;
    if(full_->remote_receiver)full_->remote_receiver->Shutdown(context->Env);
    if(full_->gimbal_client)full_->gimbal_client->Shutdown(context->Env);
    full_->menu.Shutdown();
    if (full_->bridge) context->Env->DeleteGlobalRef(full_->bridge);
    if (full_->activity) context->Env->DeleteGlobalRef(full_->activity);
    full_.reset();
}

void QuestNewtonApp::SyncFullInput(OVRFW::ovrApplFrameIn& in) {
    auto& f = *full_;
    pending_input_ = {};
    f.raw = {};
    f.head_valid = f.left_aim_valid = f.right_aim_valid = false;
    const auto sample_time = ToXrTime(in.PredictedDisplayTime);
    pending_input_.stage_valid = StageSpace != XR_NULL_HANDLE && CurrentSpace == StageSpace &&
        sample_time >= calibration_not_before_;
    if (!input_actions_attached_) return;
    XrActiveActionSet active{BaseActionSet, XR_NULL_PATH};
    XrActionsSyncInfo sync{};
    sync.type = XR_TYPE_ACTIONS_SYNC_INFO;
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    const auto result = xrSyncActions(Session, &sync);
    if (!CheckInputResult("full xrSyncActions", result)) return;
    pending_input_.focused = Focused && result == XR_SUCCESS;
    if (!pending_input_.focused) return;

    auto read_float = [&](InputId id, XrAction action, XrPath hand) {
        XrActionStateGetInfo get{};
        get.type = XR_TYPE_ACTION_STATE_GET_INFO; get.action = action; get.subactionPath = hand;
        XrActionStateFloat value{}; value.type = XR_TYPE_ACTION_STATE_FLOAT;
        if (CheckInputResult("full float action", xrGetActionStateFloat(Session, &get, &value)))
            f.raw[static_cast<std::size_t>(id)] = {value.currentState, value.isActive == XR_TRUE};
    };
    auto read_button = [&](InputId id, XrAction action, XrPath hand) {
        XrActionStateGetInfo get{};
        get.type = XR_TYPE_ACTION_STATE_GET_INFO; get.action = action; get.subactionPath = hand;
        XrActionStateBoolean value{}; value.type = XR_TYPE_ACTION_STATE_BOOLEAN;
        if (CheckInputResult("full boolean action", xrGetActionStateBoolean(Session, &get, &value)))
            f.raw[static_cast<std::size_t>(id)] = {value.currentState ? 1.F : 0.F, value.isActive == XR_TRUE};
    };
    read_float(InputId::RightTrigger, IndexTriggerAction, RightHandPath);
    read_float(InputId::LeftTrigger, IndexTriggerAction, LeftHandPath);
    read_float(InputId::RightSqueeze, GripTriggerAction, RightHandPath);
    read_float(InputId::LeftSqueeze, GripTriggerAction, LeftHandPath);
    read_button(InputId::A, ButtonAAction, RightHandPath);
    read_button(InputId::B, ButtonBAction, RightHandPath);
    read_button(InputId::X, ButtonXAction, LeftHandPath);
    read_button(InputId::Y, ButtonYAction, LeftHandPath);
    read_button(InputId::LeftMenu, ButtonMenuAction, LeftHandPath);
    read_button(InputId::LeftStickClick, thumbstickClickAction, LeftHandPath);
    read_button(InputId::RightStickClick, thumbstickClickAction, RightHandPath);
    auto locate = [&](XrSpace space, kin::Pose& pose, XrSpaceLocationFlags& flags) {
        flags = 0;
        if (space == XR_NULL_HANDLE || CurrentSpace == XR_NULL_HANDLE) return false;
        XrSpaceLocation location{}; location.type = XR_TYPE_SPACE_LOCATION;
        if (!CheckInputResult("full xrLocateSpace", xrLocateSpace(space, CurrentSpace, sample_time, &location))) return false;
        flags = location.locationFlags; pose = FromXr(location.pose);
        return Tracked(flags);
    };
    auto pose_active = [&](XrAction action, XrPath hand) {
        XrActionStateGetInfo get{};
        get.type = XR_TYPE_ACTION_STATE_GET_INFO; get.action = action; get.subactionPath = hand;
        XrActionStatePose pose{}; pose.type = XR_TYPE_ACTION_STATE_POSE;
        return CheckInputResult("full pose action", xrGetActionStatePose(Session, &get, &pose)) && pose.isActive == XR_TRUE;
    };
    XrSpaceLocationFlags flags = 0;
    pending_input_.pose_active = pose_active(GripPoseAction, RightHandPath);
    locate(RightControllerGripSpace, pending_input_.stage_from_grip, flags);
    pending_input_.position_valid = (flags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
    pending_input_.orientation_valid = (flags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
    pending_input_.position_tracked = (flags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
    pending_input_.orientation_tracked = (flags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) != 0;
    f.head_valid = locate(GetHeadSpace(), f.head, flags);
    kin::Pose aim;
    f.left_aim_valid = pose_active(AimPoseAction, LeftHandPath) && locate(LeftControllerAimSpace, aim, flags);
    if (f.left_aim_valid) f.left_aim = ToOvr(aim);
    f.right_aim_valid = pose_active(AimPoseAction, RightHandPath) && locate(RightControllerAimSpace, aim, flags);
    if (f.right_aim_valid) f.right_aim = ToOvr(aim);
}

void QuestNewtonApp::SuspendFullInput(bool reference_changed) {
    if (!full_) return;
    auto& f = *full_;
    // A brief focus loss can occur entirely between XR updates. Consume the
    // camera clutch here as well as in per-frame eligibility checks.
    f.gimbal_aim.Reset();
    if(f.gimbal_client)f.gimbal_client->Submit({GimbalOperation::Hold,false,0,0,std::chrono::steady_clock::now()});
    f.object_placement.Cancel();
    if (launch_options_.mode == verification::RunMode::benchmark && verification_running_.load())
        verification_interrupted_.store(true);
    if (reference_changed) {
        f.remote_view.Invalidate();f.last_remote_status=-1;
        f.registration.Invalidate(); ++f.reference_counter;
        f.room_scene.Invalidate(); f.room_refresh_pending=true;
    }
    ++f.suspension_counter;
    f.calibration_event.reset();
    f.registration.Update(f.head, false, false, f.settings);
    f.menu_down = f.calibration_down = true;
    f.was_suppressed = true;
    const std::lock_guard lock(f.mutex);
    f.room_ready=false;
    f.input.controller.focused = false;
    f.input.suspension = f.suspension_counter;
    f.input.reference = f.reference_counter;
    f.input.calibration_event.reset();
    f.trace.AppendObservation({InteractionObservationKind::Loss,{f.reference_counter,f.suspension_counter,0,0},{},reference_changed?1U:0U},InteractionNowNs());
    if (reference_changed) f.input.registered = false;
}

void QuestNewtonApp::UpdateRoomEnvironment(const OVRFW::ovrApplFrameIn& in) {
    auto& f=*full_;
    f.room_scene.SetEnabled(f.settings.environment_collisions);
    const bool tracked=pending_input_.focused && pending_input_.stage_valid && f.head_valid && f.registration.Valid();
    if (f.room_refresh_pending && tracked && f.settings.environment_collisions) {
        f.room_scene.Refresh(false);f.room_refresh_pending=false;
    }
    const double now=WallNow();
    if (f.settings.environment_collisions && f.room_api_available &&
        (f.room_permission_poll<0 || now-f.room_permission_poll>=1.)) {
        f.room_permission_poll=now;
        const auto* context=GetContext();
        f.room_permission_granted=false;
        if (f.bridge && f.room_permission_query) {
            f.room_permission_granted=context->Env->CallStaticBooleanMethod(f.bridge,f.room_permission_query,context->ActivityObject)==JNI_TRUE;
            if (context->Env->ExceptionCheck()) {context->Env->ExceptionClear();f.room_permission_granted=false;}
        }
        if (!f.room_permission_granted && !f.room_permission_requested && f.bridge && f.room_permission_request) {
            f.room_permission_requested=true;
            context->Env->CallStaticVoidMethod(f.bridge,f.room_permission_request,context->ActivityObject);
            if (context->Env->ExceptionCheck()) context->Env->ExceptionClear();
        }
    }
    f.room_scene.Update(ToXrTime(in.PredictedDisplayTime),StageSpace,f.registration.StageFromWorld(),
                        f.head,tracked,f.room_permission_granted);
    const auto& room=f.room_scene.Snapshot();
    const std::lock_guard lock(f.mutex);
    if (f.desired_room->revision!=room.revision || f.desired_room->enabled!=room.enabled) {
        f.desired_room=std::make_shared<const RoomEnvironment>(room);
        f.room_apply_error.clear();
    }
    f.room_ready=f.room_scene.Ready() && tracked;
    f.status.room=f.room_scene.Status();
    if (f.settings.environment_collisions) {
        if (!f.room_api_available) f.status.room="Room mapping is unavailable on this runtime";
        else if (!f.room_permission_granted) f.status.room="Spatial data permission required; toggle off/on to retry";
        else if (!f.room_apply_error.empty()) f.status.room="Room update failed: "+f.room_apply_error;
        else if (f.room_ready) {
            if (f.room_applied_enabled && f.room_applied_revision==room.revision)
                f.status.room="Room collisions active: "+std::to_string(room.colliders.size())+" surfaces";
            else f.status.room="Applying room surfaces; simulation is paused";
        }
    } else if (f.room_applied_enabled) f.status.room="Disabling room collisions";
}

void QuestNewtonApp::UpdateFullRuntime(const OVRFW::ovrApplFrameIn& in) {
    auto& f = *full_;
    if (launch_options_.mode == verification::RunMode::benchmark) {
        const bool valid = pending_input_.focused && pending_input_.stage_valid && f.head_valid;
        if (valid && !f.benchmark_placed) {
            f.registration.Update(f.head, true, false, f.settings);
            f.benchmark_stage_from_world = f.registration.StageFromWorld();
            f.benchmark_world_from_base = f.registration.WorldFromBase();
            f.benchmark_world_from_base.position = {.7F, 0.F, std::max(.1F, f.head.position[1] - .5F)};
            f.benchmark_placed = true;
        }
        if (!valid && verification_running_.load()) verification_interrupted_.store(true);
        const std::lock_guard lock(f.mutex);
        f.input = {pending_input_, {}, f.benchmark_stage_from_world, f.benchmark_world_from_base,
            valid && f.benchmark_placed, false, 0, f.reference_counter, f.suspension_counter,
            std::chrono::steady_clock::now(),{}};
        return;
    }
    if (!f.startup_menu_shown && f.head_valid) {
        f.menu.SetVisible(true, ToOvr(f.head));
        f.startup_menu_shown = true;
    }
    const auto actions = ResolveActions(f.settings, f.raw);
    const auto action = [&](SimAction id) { return actions[static_cast<std::size_t>(id)]; };
    const auto menu = ObjectPlacementMenu(f.object_placement.Active(),f.settings,actions,f.raw);
    const bool menu_down = menu.active && menu.value > .5F;
    if (menu_down && !f.menu_down && f.head_valid) {
        if(f.object_placement.Active()){f.object_placement.Cancel();f.menu.SetVisible(true,ToOvr(f.head));}
        else f.menu.Toggle(ToOvr(f.head));
    }
    f.menu_down = !menu.active || menu_down;
    const auto arm = action(SimAction::ArmEngage), calibrate = action(SimAction::Calibrate);
    pending_input_.trigger = arm.value; pending_input_.trigger_active = arm.active;
    pending_input_.calibrate_active = calibrate.active;
    pending_input_.calibrate_pressed = calibrate.value > .5F;
    const bool calibration_down = calibrate.active && calibrate.value > .5F;
    const bool calibration_press=calibration_down && !f.calibration_down;
    f.calibration_down = !calibrate.active || calibration_down;

    const auto& left_click = f.raw[static_cast<std::size_t>(InputId::LeftTrigger)];
    const auto& right_click = f.raw[static_cast<std::size_t>(InputId::RightTrigger)];
    if(f.placement_release_pending && PlacementReleaseObserved(right_click))f.placement_release_pending=false;
    f.menu.Update(in, f.left_aim, f.left_aim_valid, left_click.active && left_click.value > .5F,
                  f.right_aim, f.right_aim_valid, right_click.active && right_click.value > .5F,
                  pending_input_.focused);
    bool paused,room_paused;
    {
        const std::lock_guard lock(f.mutex); paused = f.status.paused;
        RoomEnvironment applied;applied.enabled=f.room_applied_enabled;applied.revision=f.room_applied_revision;
        const bool frame_current=pending_input_.focused && pending_input_.stage_valid && f.head_valid && f.registration.Valid();
        room_paused=RoomPhysicsBlocked(f.settings.environment_collisions,f.room_scene.Ready(),frame_current,
                                      f.room_scene.Snapshot(),&applied);
    }
    const bool controls_suppressed = ControlsSuppressed(pending_input_,paused,f.menu.IsOpen() || f.object_placement.Active() || f.placement_release_pending || f.remote_inspecting);
    const bool suppressed=controls_suppressed || room_paused;
    if (suppressed && !f.was_suppressed) {++f.suspension_counter;f.calibration_event.reset();}
    f.was_suppressed = suppressed;
    const auto clutch = action(SimAction::HeightClutch);
    f.registration.Update(f.head,RegistrationTracking(f.registration.Valid(),f.head_valid,pending_input_.focused,
        pending_input_.stage_valid,controls_suppressed,room_paused),clutch.active && clutch.value > .5F,f.settings);
    UpdateRoomEnvironment(in);
    UpdateRemoteScene();
    UpdateGimbalControl();
    std::optional<ObjectEdit> confirmed_edit;
    const bool was_placing=f.object_placement.Active();
    if(was_placing){
        const auto scene=f.scenes.Read();
        bool room_available;
        std::optional<std::uint32_t> ignored_body;
        {
            const std::lock_guard lock(f.mutex);
            const bool frame_current=f.registration.Valid() && pending_input_.focused && pending_input_.stage_valid && f.head_valid;
            room_available=!f.fault && !RoomPhysicsBlocked(f.settings.environment_collisions,f.room_ready,frame_current,*f.desired_room,scene.room.get());
            if(f.object_placement.Edit().kind==ObjectEditKind::Move){
                if(f.details.model_generation==scene.model_generation)
                    for(const auto& object:f.details.objects)if(object.id==f.object_placement.Edit().id)ignored_body=object.body_index;
                room_available=room_available && ignored_body.has_value();
            }
        }
        const RoomEnvironment ordinary_ground;
        const auto& applied=scene.room?*scene.room:ordinary_ground;
        const bool tracking=room_available && pending_input_.focused && pending_input_.stage_valid && f.head_valid && f.right_aim_valid;
        auto preview=tracking?CubePlacement({{f.right_aim.Translation.x,f.right_aim.Translation.y,f.right_aim.Translation.z},
            {f.right_aim.Rotation.x,f.right_aim.Rotation.y,f.right_aim.Rotation.z,f.right_aim.Rotation.w}},
            f.registration.StageFromWorld(),applied,f.object_placement.Edit().half_extents,&scene,ignored_body):std::nullopt;
        auto edit=f.object_placement.Update(preview,tracking,right_click.active,right_click.value,f.reference_counter,
            f.settings_generation,applied.revision,f.suspension_counter);
        if(edit){
            // Retire intent without pretending healthy room/head tracking was lost.
            // The matching input epoch and command are published together below.
            ++f.suspension_counter;f.calibration_event.reset();f.menu_down=f.calibration_down=true;f.was_suppressed=true;
            f.placement_release_pending=true;
            edit->suspension=f.suspension_counter;
            confirmed_edit=edit;
        }else if(!f.object_placement.Active()){
            SuspendFullInput(false);
            const std::lock_guard lock(f.mutex);f.status.diagnostic="Placement cancelled after tracking or scene change";
        }
    }
    SimMenuStatus status;
    bool fault;
    {
        const std::lock_guard lock(f.mutex);
        f.input = {pending_input_, actions, f.registration.StageFromWorld(), f.registration.WorldFromBase(),
            f.registration.Valid(), f.menu.IsOpen() || was_placing || f.placement_release_pending || f.remote_inspecting, f.calibration_counter, f.reference_counter,
            f.suspension_counter, std::chrono::steady_clock::now(),f.calibration_event};
        if(confirmed_edit){
            if(f.object_edits.size()<16){f.object_edits.push_back(*confirmed_edit);f.selected_object=confirmed_edit->id;f.status.diagnostic="Cube edit queued; release trigger to rearm";}
            else f.status.diagnostic="Object queue busy; place the cube again";
            wake_worker_.notify_all();
        }
        if (calibration_press && !was_placing) {
            RoomEnvironment applied;applied.enabled=f.room_applied_enabled;applied.revision=f.room_applied_revision;
            const bool current_room_pause=RoomPhysicsBlocked(f.settings.environment_collisions,f.room_ready,
                RoomFrameCurrent(f.input,f.input.sampled),*f.desired_room,&applied);
            auto event=CaptureCalibration(f.input,f.settings_generation,paused,room_paused || current_room_pause,f.input.sampled);
            if(event && f.settings.floating_base) {
                const auto measured=f.scenes.Read();
                if(measured.bodies.empty())event.reset();
                else event->world_from_base=measured.bodies.front();
            }
            if(event) {
                f.calibration_event=std::move(event);++f.calibration_counter;
                f.input.calibration=f.calibration_counter;f.input.calibration_event=f.calibration_event;
            }
        }
        f.status.objects="Selected cube "+(f.selected_object?std::to_string(f.selected_object):std::string("none"))+
            " | "+std::to_string(std::count_if(f.details.objects.begin(),f.details.objects.end(),[](const auto& object){return object.id!=0;}))+" / 8 user cubes";
        f.status.placing_object=f.object_placement.Active();
        if(f.status.placing_object){
            const std::string cancel=f.settings.bindings[static_cast<std::size_t>(SimAction::Menu)]==InputId::RightTrigger?"Left Menu cancels.":"Menu cancels.";
            f.status.diagnostic=(f.object_placement.Preview()?"Green cube: release then press right index to place. ":
                "Aim at a floor or tabletop with space for the whole cube. ")+cancel;
        }
        status = f.status; fault = f.fault;
        f.trace.AppendInput({f.head,f.head_valid,f.raw,f.input,f.settings,f.settings_generation,++f.input_sequence},InteractionNowNs());
    }
    if (fault && !f.fault_menu_shown && f.head_valid) {
        f.menu.SetVisible(true, ToOvr(f.head)); f.fault_menu_shown = true;
        SuspendFullInput(false);
    } else if (!fault) f.fault_menu_shown = false;
    if (!pending_input_.stage_valid) status.diagnostic = "STAGE floor unavailable; waiting for valid room reference";
    if (!fault && f.settings.environment_collisions) {
        const std::lock_guard lock(f.mutex);
        if (!f.room_ready || !f.room_applied_enabled || f.room_applied_revision!=f.desired_room->revision ||
            !f.room_apply_error.empty()) status.diagnostic="Simulation paused for room collisions. "+status.room;
    }
    const double now = WallNow();
    // Histogram summaries copy 160 KB each; read at 2 Hz, never each frame.
    if (now - f.last_diagnostics_time >= .5) {
        const auto gpu = render_gpu_timing_->Snapshot();
        const auto cpu = render_cpu_timing_->Snapshot();
        if(!f.remote_inspecting){
            status.gpu = gpu.available ? "GPU mean " + std::to_string(gpu.mean_us / 1000.) + " ms" : "GPU: pending";
            if (cpu.available) status.gpu += " / XR CPU " + std::to_string(cpu.mean_us / 1000.) + " ms";
        }
        {
            const std::lock_guard lock(f.mutex); f.status.gpu = status.gpu;
            status.latency="Input age "+(f.input_age_ms>=0?std::to_string(static_cast<int>(f.input_age_ms)):std::string("?"))+
                " ms | scene age "+(f.scene_age_ms>=0?std::to_string(static_cast<int>(f.scene_age_ms)):std::string("?"))+" ms";
            f.status.latency=status.latency;
        }
        f.last_diagnostics_time = now;
    }
    f.menu.SetStatus(status);
}

void QuestNewtonApp::RenderFullRuntime(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out) {
    auto& f = *full_;
    const auto snapshot = f.scenes.Read();
    const bool benchmark = launch_options_.mode == verification::RunMode::benchmark;
    if(!benchmark && RenderRemoteScene(in,out))return;
    if (benchmark && Focused && pending_input_.stage_valid && f.head_valid &&
        std::abs(refresh_hz_.load() - 90.0F) < .1F) {
        verification_frame_available_.store(true);
        wake_worker_.notify_all();
    }
    if (snapshot.publication && (benchmark ? f.benchmark_placed : f.registration.Valid())) {
        const auto& pose = benchmark ? f.benchmark_stage_from_world : f.registration.StageFromWorld();
        const auto world = PoseMatrix({pose.position, pose.rotation});
        if (snapshot.publication != last_render_generation_ || world != placement_) {
            BodySnapshot arm;
            arm.valid = true; arm.generation = snapshot.publication;
            for (std::size_t i = 0; i < kArmBodies; ++i) {
                std::copy(snapshot.bodies[i].position.begin(), snapshot.bodies[i].position.end(), arm.values.begin()+i*7);
                std::copy(snapshot.bodies[i].rotation.begin(), snapshot.bodies[i].rotation.end(), arm.values.begin()+i*7+3);
            }
            if (!renderer_.Update(arm, world)) { Fail("full_renderer", "invalid arm snapshot; previous pose retained"); return; }
            placement_ = world; last_render_generation_ = snapshot.publication;
        }
        renderer_.SetOpacity(f.settings.opacity);
        const bool show_room=f.settings.show_room_surfaces && f.settings.environment_collisions &&
            f.room_scene.Ready() && snapshot.room && snapshot.room->revision==f.room_scene.Snapshot().revision;
        f.environment.Append(snapshot, world, f.settings.show_grid, show_room, out.Surfaces);
        if(!benchmark){
            std::optional<kin::Pose> target,measured;
            std::optional<CubeMarker> selected;
            std::vector<ContactPoint> contacts;
            {
                const std::lock_guard lock(f.mutex);
                const auto now=std::chrono::steady_clock::now();
                f.scene_age_ms=std::chrono::duration<double,std::milli>(now-snapshot.published_at).count();
                if(f.selected_object && f.details.model_generation==snapshot.model_generation){
                    for(const auto& object:f.details.objects)if(object.id==f.selected_object && object.body_index<snapshot.bodies.size())
                        selected=CubeMarker{snapshot.bodies[object.body_index],object.half_extents};
                }
                if(f.show_diagnostics && !f.fault && f.target_reference==f.reference_counter &&
                   f.target_generation==snapshot.model_generation && now>=f.target_sampled &&
                   now-f.target_sampled<=std::chrono::milliseconds(150) && now>=snapshot.published_at &&
                   now-snapshot.published_at<=std::chrono::milliseconds(150) && pending_input_.focused && pending_input_.stage_valid){
                    target=f.world_target;
                    if(snapshot.bodies.size()>9)measured=snapshot.bodies[9];
                }
                if(f.show_diagnostics && !f.fault && f.details_have_contacts && f.details_reference==f.reference_counter &&
                   f.details.model_generation==snapshot.model_generation && f.details.step_index<=snapshot.step_index &&
                   now>=f.details_sampled && now-f.details_sampled<=std::chrono::milliseconds(150) &&
                   pending_input_.focused && pending_input_.stage_valid && (!f.settings.environment_collisions || f.room_ready))
                    contacts=f.details.contacts;
            }
            f.environment.AppendDiagnostics(world,target,measured,f.object_placement.Preview(),f.object_placement.Edit().half_extents,contacts,selected,out.Surfaces);
            f.trace.AppendObservation({InteractionObservationKind::Render,{snapshot.publication,snapshot.step_index,snapshot.model_generation,0},
                {std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-snapshot.published_at).count(),0,0,0},0},InteractionNowNs());
        }
        const auto before = out.Surfaces.size();
        renderer_.Append(out.FrameMatrices.CenterView, out.Surfaces);
        rendered_body_count_ = out.Surfaces.size() - before;
    }
    if (!benchmark) f.menu.Render(in, out);
}

void QuestNewtonApp::RunFullSimulation() {
    if (launch_options_.mode == verification::RunMode::benchmark) { RunFullBenchmark(); return; }
    auto& f = *full_;
    std::unique_ptr<PythonSession> session;
    FullControlMachine control;
    control.SetObserver([&](const FullControlEvent& event){f.trace.AppendControl(event,InteractionNowNs());});
    SimulationClock clock;
    SimSettings settings;
    SceneSnapshot latest;
    std::shared_ptr<const RoomEnvironment> applied_room;
    const auto decode=[&](const std::vector<std::byte>& bytes) {
        auto snapshot=Decode(bytes);snapshot.room=applied_room;return snapshot;
    };
    SceneDetailsPoll details_poll;
    bool trace_save_failed=false;
    bool paused = false, faulted = false;
    double input_age_ms=-1;
    bool environment_blocked=false,room_apply_failed=false;
    std::uint64_t failed_room_revision=0;
    std::uint64_t applied_settings = 0, calibration = 0, reference = 0, suspension = 0;
    kin::Pose applied_base;
    double diagnostic_wall = WallNow(), diagnostic_sim = 0;
    double report_wall = 0;
    const double worker_started = WallNow();
    const auto save_trace=[&](bool retry=false) {
        if(!f.trace.NeedsFlush() || (trace_save_failed && !retry))return;
        const auto trace_status=f.trace.Status();std::string error;
        const auto path=std::filesystem::path(internal_data_path_)/(trace_status.run_id+".qitr");
        const bool saved=f.trace.Flush(path,error);
        trace_save_failed=!saved;
        clock.Accumulate(WallNow(),true);
        const std::lock_guard lock(f.mutex);
        f.status.recording=saved?(trace_status.loss?"Saved incomplete trace; replay unavailable":"Saved "+trace_status.run_id+".qitr"):
            "Save failed; Stop and save retries: "+error;
        if(saved)FullLog(ANDROID_LOG_INFO,"QUEST_INTERACTION_SAVED file="+path.filename().string()+" events="+
            std::to_string(trace_status.event_count)+" loss="+std::to_string(trace_status.loss));
    };
    const auto report = [&] {
        const double now=WallNow();
        if(now-report_wall<.5)return;
        report_wall=now;
        std::string diagnostic,room_status,remote_stats;
        bool pending,status_fault;
        { const std::lock_guard lock(f.mutex);diagnostic=f.status.diagnostic;pending=f.status.settings_pending;status_fault=f.fault;room_status=f.status.room;remote_stats=f.remote_stats_json; }
        const auto timing=physics_timing_->Snapshot();
        const auto gpu=render_gpu_timing_->Snapshot();
        const auto cpu=render_cpu_timing_->Snapshot();
        std::string gripper_status="Gripper: starting";
        if(session){
            try {gripper_status=session->GripperStatus();}
            catch(const std::exception&) {gripper_status="Gripper diagnostics unavailable";}
        }
        { const std::lock_guard lock(f.mutex); f.status.gripper=gripper_status; }
        std::ostringstream out;out.imbue(std::locale::classic());out << std::setprecision(17) << std::boolalpha;
        out << "{\"schema_version\":1,\"pid\":" << getpid() << ",\"worker_started\":" << worker_started
            << ",\"monotonic_s\":" << now << ",\"ready\":" << (session && !faulted && !status_fault && latest.step_index>0)
            << ",\"faulted\":" << (faulted || status_fault) << ",\"worker_fault\":" << faulted
            << ",\"status_fault\":" << status_fault << ",\"paused\":" << paused << ",\"settings_pending\":" << pending
            << ",\"settings\":" << SettingsJson(settings) << ",\"diagnostic\":" << JsonString(diagnostic)
            << ",\"step_index\":" << latest.step_index << ",\"sim_time\":" << latest.simulation_time
            << ",\"model_generation\":" << latest.model_generation << ",\"contacts\":" << latest.contact_count
            << ",\"environment_applied\":" << (applied_room && applied_room->enabled)
            << ",\"environment_blocked\":" << environment_blocked
            << ",\"environment_revision\":" << (applied_room?applied_room->revision:0)
            << ",\"environment_surfaces\":" << (applied_room?applied_room->colliders.size():0)
            << ",\"room_status\":" << JsonString(room_status)
            << ",\"remote_render\":"<<remote_stats
            << ",\"body_count\":" << latest.bodies.size() << ",\"arm_engaged\":" << control.Current().engaged
            << ",\"gripper_target\":" << control.Gripper() << ",\"dropped_wall_seconds\":" << clock.DroppedWallSeconds()
            << ",\"gripper_control_status\":" << JsonString(gripper_status)
            << ",\"physics_samples\":" << timing.count << ",\"physics_mean_us\":" << timing.mean_us
            << ",\"physics_p95_us\":" << timing.p95_us << ",\"physics_p99_us\":" << timing.p99_us
            << ",\"physics_max_us\":" << timing.max_us << ",\"gpu_mean_us\":" << gpu.mean_us
            << ",\"gpu_available\":" << gpu.available << ",\"xr_cpu_mean_us\":" << cpu.mean_us << ",\"body_q\":[";
        bool first=true;
        for(const auto& body:latest.bodies){
            for(float value:body.position){if(!first)out<<',';first=false;out<<value;}
            for(float value:body.rotation){out<<','<<value;}
        }
        out << "]}";
        const auto path=std::filesystem::path(internal_data_path_)/"full-runtime-status.json";
        auto temporary=path;temporary+=".pending";
        { std::ofstream file(temporary,std::ios::binary|std::ios::trunc);file<<out.str();file.close();
          if(!file){FullLog(ANDROID_LOG_ERROR,"QUEST_FULL_REPORT write failed");return;} }
        std::error_code error;std::filesystem::rename(temporary,path,error);
        if(error)FullLog(ANDROID_LOG_ERROR,"QUEST_FULL_REPORT publish failed: "+error.message());
    };
    const auto reset_control = [&] {
        control.Reset();
        clock.Reset(WallNow());
        diagnostic_wall = WallNow(); diagnostic_sim = 0;
    };
    const auto metadata = [&] {
        const auto value = session->Metadata();
        FullLog(ANDROID_LOG_INFO, "QUEST_FULL_RUNTIME " + value);
        const std::lock_guard lock(f.mutex);
        f.status.backend = "Newton SolverMuJoCo CPU / Warp CPU JIT";
    };
    while (!stop_worker_.load()) {
        FullInputFrame input;
        SimSettings requested;
        std::uint64_t generation;
        std::uint64_t batch_sequence;
        std::deque<SimMenuCommand> commands;
        std::size_t processed_commands=0;
        std::deque<ObjectEdit> object_edits;
        std::shared_ptr<const RoomEnvironment> desired_room_state;
        bool room_ready=false;
        JavaVM* vm;
        jobject activity;
        jclass bridge;
        {
            const std::lock_guard lock(f.mutex);
            input = f.input; requested = f.requested_settings; generation = f.settings_generation;
            batch_sequence=f.input_sequence;
            commands.swap(f.commands);
            object_edits.swap(f.object_edits);
            desired_room_state=f.desired_room;room_ready=f.room_ready;
            vm = f.vm; activity = f.activity; bridge = f.bridge;
        }
        const auto& desired_room=*desired_room_state;
        try {
            const auto room_frame_current=[&] {
                return RoomFrameCurrent(input,std::chrono::steady_clock::now());
            };
            environment_blocked=RoomPhysicsBlocked(requested.environment_collisions,room_ready,room_frame_current(),
                                                   desired_room,applied_room.get());
            const bool retry = std::find(commands.begin(), commands.end(), SimMenuCommand::Reset) != commands.end();
            if (retry && faulted) {
                session.reset(); faulted = false;
                { const std::lock_guard lock(f.mutex); f.fault = false; f.status.diagnostic = "Retrying full runtime"; }
            }
            for (const auto command : commands) if (command == SimMenuCommand::TogglePause) {
                paused = !paused;
                control.Suppress();
                const std::lock_guard lock(f.mutex);
                f.status.paused = paused;
            }
            if (!faulted && !session && input.registered) {
                if (!vm || !activity || !bridge) throw std::runtime_error("Java bridge unavailable; Reset retries initialization");
                { const std::lock_guard lock(f.mutex); f.status.diagnostic = "Starting Python, Warp CPU JIT and Newton scene"; }
                settings = requested;
                applied_room.reset();room_apply_failed=false;
                details_poll.BeginSession();
                {
                    const std::lock_guard lock(f.mutex);
                    f.status.settings_pending=true;
                    f.room_applied_enabled=false;f.room_applied_revision=0;f.room_apply_error.clear();
                    f.details={};f.details_sampled={};f.details_have_contacts=false;f.details_error.clear();
                    f.selected_object=0;f.world_target.reset();f.target_sampled={};f.target_generation=0;
                    f.status.contacts="Contacts: waiting for new simulation session";
                }
                session = std::make_unique<PythonSession>(vm, activity, bridge, InitialSettings(settings, input.world_from_base));
                latest = decode(session->Snapshot());
                f.scenes.Publish(latest);
                applied_base = input.world_from_base;
                applied_settings = generation;
                clock.Configure(settings, WallNow()); reset_control();
                calibration = input.calibration; reference = input.reference; suspension = input.suspension;
                metadata();
                { const std::lock_guard lock(f.mutex);
                  f.status.settings_pending = f.settings_generation != applied_settings;
                  f.status.diagnostic = "Release arm trigger, then calibrate"; f.fault = false; }
            }
            if (session && !faulted) {
                if (generation != applied_settings) {
                    const auto old_generation = latest.model_generation;
                    session->Configure(SettingsJson(requested));
                    latest = decode(session->Snapshot());
                    settings = requested; applied_settings = generation;
                    clock.Configure(settings, WallNow());
                    if (latest.model_generation != old_generation) {
                        reset_control();
                        calibration = input.calibration;
                    }
                    f.scenes.Publish(latest);
                    metadata();
                    const std::lock_guard lock(f.mutex);
                    f.status.settings_pending = f.settings_generation != applied_settings;
                    f.status.diagnostic = "Settings applied";
                }
                if (input.reference != reference) {
                    reference = input.reference; control.Invalidate();
                }
                if (input.suspension != suspension) {
                    suspension = input.suspension;
                    // Preserve calibrated targets, but consume any intervening
                    // menu/focus/tracking loss even if it lasted one XR frame.
                    control.Suppress();
                }
                if (!paused && !environment_blocked && !settings.floating_base && !SamePose(input.world_from_base, applied_base)) {
                    bool base_current;
                    {
                        const std::lock_guard lock(f.mutex);
                        const auto now=std::chrono::steady_clock::now();
                        base_current=ValidFullInput(input,now) && !SubstepBoundaryChanged(input,applied_settings,f.input,
                            f.settings_generation,!f.commands.empty() || !f.object_edits.empty(),now) && !RoomPhysicsBlocked(f.requested_settings.environment_collisions,
                            f.room_ready,RoomFrameCurrent(f.input,now),*f.desired_room,applied_room.get());
                    }
                    if(base_current) {
                        latest = decode(session->SetBasePose(input.world_from_base));
                        applied_base = input.world_from_base;
                    }
                }
                for (;processed_commands<commands.size();++processed_commands) {
                    const auto command=commands[processed_commands];
                    if(command==SimMenuCommand::StopRecording){f.trace.Stop(InteractionNowNs());save_trace(true);continue;}
                    if(command==SimMenuCommand::RecordReset){
                        f.trace.Stop(InteractionNowNs());save_trace(true);
                        std::string error;const auto stamp=InteractionNowNs();
                        if(!f.trace.Start("interaction_"+std::to_string(stamp),settings,stamp,error)){
                            const std::lock_guard lock(f.mutex);f.status.recording="Recording failed: "+error;continue;
                        }
                        trace_save_failed=false;
                    }
                    f.trace.AppendObservation({InteractionObservationKind::Command,{static_cast<std::uint64_t>(command),generation,input.reference,0},{},0},InteractionNowNs());
                    const bool explicit_reset=command==SimMenuCommand::Reset || command==SimMenuCommand::RecordReset;
                    const char* name = explicit_reset ? "reset" :
                        command == SimMenuCommand::SpawnBox ? "spawn_box" :
                        command == SimMenuCommand::RemoveBox ? "remove_box" : nullptr;
                    if (name) {
                        const auto old_generation = latest.model_generation;
                        latest = decode(session->Command(name));
                        if (explicit_reset || latest.model_generation != old_generation) {
                            reset_control(); calibration = input.calibration;
                        }
                        f.scenes.Publish(latest);
                        const std::lock_guard lock(f.mutex);
                        f.status.diagnostic = std::string(name) + " applied; release trigger and recalibrate";
                        if (explicit_reset) f.fault = false;
                    }
                }
                const bool room_change=settings.environment_collisions
                    ? (!applied_room || !applied_room->enabled || applied_room->revision!=desired_room.revision)
                    : (applied_room && applied_room->enabled);
                const bool room_batch_available=settings.environment_collisions
                    ? (room_ready && desired_room.enabled && room_frame_current()) : !desired_room.enabled;
                if (room_change && room_batch_available &&
                    (!room_apply_failed || failed_room_revision!=desired_room.revision)) {
                    try {
                        auto next=Decode(session->SetEnvironment(RoomEnvironmentJson(desired_room)));
                        applied_room=desired_room_state;
                        next.room=applied_room;latest=std::move(next);f.scenes.Publish(latest);
                        room_apply_failed=false;
                        const std::lock_guard lock(f.mutex);
                        f.room_applied_revision=applied_room->revision;f.room_applied_enabled=applied_room->enabled;
                        f.room_apply_error.clear();
                    } catch (const std::exception& error) {
                        room_apply_failed=true;failed_room_revision=desired_room.revision;
                        const std::lock_guard lock(f.mutex);f.room_apply_error=error.what();
                    }
                    // Scene editing is a paused interval, not catch-up work. Keep
                    // the clock's step phase, simulation time and existing drops.
                    clock.Accumulate(WallNow(),true);
                }
                environment_blocked=RoomPhysicsBlocked(settings.environment_collisions,room_ready,room_frame_current(),
                                                       desired_room,applied_room.get());
                if (settings.environment_collisions || (applied_room && applied_room->enabled)) {
                    const std::lock_guard lock(f.mutex);
                    environment_blocked=environment_blocked || f.settings_generation!=applied_settings ||
                        f.desired_room->revision!=desired_room.revision ||
                        (settings.environment_collisions && !f.room_ready);
                }
                if (environment_blocked) {
                    control.Suppress();
                }
                for(const auto& edit:object_edits){
                    bool current;
                    std::optional<std::uint32_t> ignored_body;
                    {
                        const std::lock_guard lock(f.mutex);
                        const auto now=std::chrono::steady_clock::now();
                        current=edit.reference==f.input.reference && edit.suspension==f.input.suspension && edit.settings_generation==applied_settings &&
                            f.settings_generation==applied_settings && edit.room_revision==(applied_room?applied_room->revision:0) &&
                            RoomFrameCurrent(f.input,now) && !RoomPhysicsBlocked(f.requested_settings.environment_collisions,
                            f.room_ready,RoomFrameCurrent(f.input,now),*f.desired_room,applied_room.get());
                        if(edit.kind==ObjectEditKind::Move){
                            if(f.details.model_generation==latest.model_generation)
                                for(const auto& object:f.details.objects)if(object.id==edit.id)ignored_body=object.body_index;
                            current=current && ignored_body.has_value();
                        }
                    }
                    if(!current){const std::lock_guard lock(f.mutex);f.status.diagnostic="Cube edit cancelled: scene or tracking changed";continue;}
                    const RoomEnvironment ordinary_ground;
                    if((edit.kind==ObjectEditKind::Spawn || edit.kind==ObjectEditKind::Move) &&
                       !CubePlacementClear(edit.pose,edit.half_extents,applied_room?*applied_room:ordinary_ground,&latest,ignored_body)){
                        const std::lock_guard lock(f.mutex);f.status.diagnostic="Cube placement is occupied or no longer valid; choose another position";continue;
                    }
                    try {
                        f.trace.AppendObservation({InteractionObservationKind::Command,{100U+static_cast<std::uint64_t>(edit.kind),edit.id,edit.reference,edit.room_revision},
                            {edit.pose.position[0],edit.pose.position[1],edit.pose.position[2],edit.half_extents[0]},0},InteractionNowNs());
                        auto next=decode(session->Command(ObjectEditJson(edit)));
                        latest=std::move(next);f.scenes.Publish(latest);details_poll.Invalidate();
                        const std::lock_guard lock(f.mutex);f.status.diagnostic="Cube edited; robot state and simulation time preserved";
                    }catch(const std::exception& error){
                        // A rejected object transaction keeps the previous physics session.
                        const std::lock_guard lock(f.mutex);f.status.diagnostic=std::string("Cube edit failed: ")+error.what();
                    }
                    clock.Accumulate(WallNow(),true);
                }
                // Consume a discrete press promptly using its captured pose,
                // never a later pose at the next decimated IK update.
                if(input.calibration!=calibration) {
                    calibration=input.calibration;
                    bool eligible=false;
                    if(input.calibration_event) {
                        const std::lock_guard lock(f.mutex);
                        const auto now=std::chrono::steady_clock::now();
                        const bool room_pause=RoomPhysicsBlocked(f.requested_settings.environment_collisions,f.room_ready,
                            RoomFrameCurrent(f.input,now),*f.desired_room,applied_room.get());
                        eligible=f.commands.empty() && f.object_edits.empty() && f.settings_generation==applied_settings &&
                            CalibrationIsCurrent(*input.calibration_event,f.input,applied_settings,paused,room_pause,now);
                    }
                    if(eligible) {
                        const auto& event=*input.calibration_event;
                        auto sample=event.controller;sample.calibrate_pressed=false;
                        control.Calibrate(sample,kin::Compose(event.stage_from_world,event.world_from_base),HandOffsetPose(settings.hand_offset_degrees));
                    }
                }
                const auto count = clock.Accumulate(WallNow(), paused || environment_blocked);
                f.trace.AppendObservation({InteractionObservationKind::Clock,{count,latest.step_index,batch_sequence,0},
                    {clock.SimulationTime(),clock.DroppedWallSeconds(),settings.physics_dt,0},(paused?1U:0U)|(environment_blocked?2U:0U)},InteractionNowNs());
                for (unsigned i=0; i<count && !stop_worker_.load(); ++i) {
                    bool boundary_changed;
                    {
                        const std::lock_guard lock(f.mutex);
                        const auto now=std::chrono::steady_clock::now();
                        environment_blocked=RoomPhysicsBlocked(f.requested_settings.environment_collisions,f.room_ready,
                            RoomFrameCurrent(f.input,now),*f.desired_room,applied_room.get());
                        boundary_changed=environment_blocked || SubstepBoundaryChanged(input,applied_settings,f.input,
                            f.settings_generation,!f.commands.empty() || !f.object_edits.empty(),now);
                        f.trace.AppendObservation({InteractionObservationKind::Boundary,{batch_sequence,f.input_sequence,applied_settings,f.settings_generation},
                            {std::chrono::duration<double,std::milli>(now-input.sampled).count(),0,0,0},
                            (boundary_changed?1U:0U)|(environment_blocked?2U:0U)},InteractionNowNs());
                    }
                    if(boundary_changed) {
                        control.Suppress();
                        clock.Accumulate(WallNow(),true);
                        break;
                    }
                    const auto tick = clock.Advance();
                    // Validate age every substep (a slow Python step must not
                    // make an old frame usable by later catch-up steps).
                    const bool allowed = ValidInput(input) && !paused && !environment_blocked;
                    if (!allowed && control.InputAllowed()) {
                        control.Suppress();
                    }
                    if (tick.control_due) {
                        auto sample = input.controller;
                        if (!allowed) sample.focused = false;
                        sample.calibrate_pressed = false;
                        kin::Pose world_base = settings.floating_base ? latest.bodies.front() : input.world_from_base;
                        const auto stage_base = kin::Compose(input.stage_from_world, world_base);
                        ATrace_beginSection("QuestFull.ControlIK");
                        const auto gripper = input.actions[static_cast<std::size_t>(SimAction::Gripper)];
                        control.Control(sample,gripper,stage_base,HandOffsetPose(settings.hand_offset_degrees),allowed);
                        ATrace_endSection();
                    }
                    const auto start = std::chrono::steady_clock::now();
                    input_age_ms=std::chrono::duration<double,std::milli>(start-input.sampled).count();
                    bool gripper_input_allowed;
                    {
                        // Recheck the grip action after IK as well as the main
                        // substep barrier. Losing only this action must not let
                        // an old raw closure keep advancing a limited target.
                        const std::lock_guard lock(f.mutex);
                        const auto now=std::chrono::steady_clock::now();
                        gripper_input_allowed=allowed && control.InputAllowed() && control.GripperRearmed() &&
                            GripperInputCurrent(input,f.input,now) && f.settings_generation==applied_settings &&
                            f.commands.empty() && f.object_edits.empty() &&
                            !RoomPhysicsBlocked(f.requested_settings.environment_collisions,f.room_ready,
                                RoomFrameCurrent(f.input,now),*f.desired_room,applied_room.get());
                    }
                    auto bytes = session->Step(tick.physics_dt, control.Targets(), control.Gripper(),gripper_input_allowed);
                    const auto elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
                    physics_timing_->Observe(elapsed);
                    latest = decode(bytes);
                    f.trace.AppendObservation({InteractionObservationKind::Physics,{latest.step_index,latest.model_generation,batch_sequence,latest.contact_count},
                        {tick.physics_dt,latest.simulation_time,elapsed,input_age_ms},0},InteractionNowNs());
                    if (tick.publish_due) f.scenes.Publish(latest);
                }
                const double now = WallNow();
                {
                    const std::lock_guard lock(f.mutex);
                    if(f.show_diagnostics && control.Current().calibrated){
                        kin::Pose palm;
                        // Held FK remains meaningful while released; engaged target shows IK error.
                        const auto base=settings.floating_base?latest.bodies.front():input.world_from_base;
                        if(control.Current().engaged)f.world_target=kin::Compose(base,control.Current().robot_base_from_grip);
                        else if(kin::ForwardKinematics(control.Targets(),palm))f.world_target=kin::Compose(base,palm);
                        f.target_reference=input.reference;
                        f.target_generation=latest.model_generation;f.target_sampled=std::chrono::steady_clock::now();
                    }else f.world_target.reset();
                }
                bool show_details;
                {const std::lock_guard lock(f.mutex);show_details=f.show_diagnostics;}
                if(details_poll.Due(latest.model_generation,show_details,now)){
                    const bool sample_contacts=details_poll.ContactsDue(show_details,now);
                    details_poll.Attempt(now,sample_contacts);
                    try {
                        SceneDetails details;std::string error;
                        if(!DecodeSceneDetails(session->Details(sample_contacts),latest,details,error))throw std::runtime_error(error);
                        const std::lock_guard lock(f.mutex);f.details=std::move(details);
                        f.details_sampled=std::chrono::steady_clock::now();f.details_reference=input.reference;
                        f.details_have_contacts=sample_contacts && f.details.contacts_current;f.details_error.clear();
                        details_poll.Succeeded(latest.model_generation,now);
                    }catch(const std::exception& error){
                        const std::lock_guard lock(f.mutex);f.details_have_contacts=false;f.details_error=error.what();
                        details_poll.Failed(now);
                    }
                }
                if (now - diagnostic_wall >= .5) {
                    const double achieved = (latest.simulation_time-diagnostic_sim) / (now-diagnostic_wall);
                    diagnostic_sim = latest.simulation_time; diagnostic_wall = now;
                    const std::lock_guard lock(f.mutex);
                    f.status.paused = paused;
                    f.status.physicsCpu = "Physics CPU " + std::to_string(latest.step_cpu_ms) + " ms / " +
                        std::to_string(achieved) + "x real time / dropped " + std::to_string(clock.DroppedWallSeconds()) + " s";
                    f.status.contacts = "Contacts " + std::to_string(latest.contact_count) +
                        " / step " + std::to_string(latest.step_index) + " / " + std::string(control.Current().reason);
                    f.status.control=std::string("Arm: ")+std::string(control.Current().reason)+" | grip "+std::to_string(control.Gripper());
                    f.input_age_ms=input_age_ms;
                    if(f.show_diagnostics && f.details_have_contacts && f.details.model_generation==latest.model_generation){
                        auto selected=std::find_if(f.details.objects.begin(),f.details.objects.end(),[&](const auto& object){return f.selected_object && object.id==f.selected_object;});
                        unsigned left=0,right=0,support=0;float force=0;
                        if(selected!=f.details.objects.end())for(const auto& point:f.details.contacts){
                            const auto body=static_cast<std::int32_t>(selected->body_index);
                            if(point.body_a!=body && point.body_b!=body)continue;
                            const auto other=point.body_a==body?point.body_b:point.body_a;
                            if(other==10)++left;if(other==11)++right;if(other==-1)++support;
                            force=std::max(force,std::abs(point.normal_force));
                        }
                        f.status.contacts="Cube candidates L/R/world "+std::to_string(left)+"/"+std::to_string(right)+"/"+std::to_string(support)+
                            " | peak "+std::to_string(force)+" N"+(f.details.truncated?" (truncated)":"");
                    }
                    if(!f.details_error.empty())f.status.contacts="Contact diagnostics unavailable: "+f.details_error;
                    else if(f.show_diagnostics && !f.details.contacts_current)f.status.contacts="Contacts: waiting for a physics step after base movement";
                }
            }
        } catch (const std::exception& error) {
            faulted = true; control.Suppress();
            FullLog(ANDROID_LOG_ERROR, "QUEST_FULL_FAULT worker: " + std::string(error.what()));
            const std::lock_guard lock(f.mutex);
            f.fault = true; f.status.settings_pending = true;
            f.status.diagnostic = std::string(error.what()) + " — Reset to retry";
        } catch (...) {
            faulted = true; control.Suppress();
            FullLog(ANDROID_LOG_ERROR, "QUEST_FULL_FAULT unknown worker exception");
            const std::lock_guard lock(f.mutex);
            f.fault = true; f.status.settings_pending = true;
            f.status.diagnostic = "Unknown simulation error; Reset to retry";
        }
        try { report(); } catch (const std::exception& error) {
            FullLog(ANDROID_LOG_ERROR,"QUEST_FULL_REPORT "+std::string(error.what()));
        }
        try {
            // A fault may stop command processing midway. Honour remaining Stop
            // requests without moving them ahead of earlier healthy Start commands.
            for(;processed_commands<commands.size();++processed_commands)
                if(commands[processed_commands]==SimMenuCommand::StopRecording){f.trace.Stop(InteractionNowNs());save_trace(true);}
            f.trace.Expire(InteractionNowNs());
            const auto trace_status=f.trace.Status();
            if(trace_status.recording){
                const std::lock_guard lock(f.mutex);f.status.recording="Recording "+std::to_string(trace_status.event_count)+" events (60 s maximum)";
            }
            save_trace();
        }catch(const std::exception& error){FullLog(ANDROID_LOG_ERROR,"QUEST_INTERACTION_SAVE "+std::string(error.what()));}
        // Never spin while JIT is unavailable or the simulation is paused.
        // Short bounded waits also let input age checks and Reset run promptly.
        std::unique_lock lock(wait_mutex_);
        wake_worker_.wait_for(lock, std::chrono::milliseconds(1), [this] { return stop_worker_.load(); });
    }
    try {f.trace.Stop(InteractionNowNs());save_trace(true);}
    catch(const std::exception& error){FullLog(ANDROID_LOG_ERROR,"QUEST_INTERACTION_SAVE shutdown: "+std::string(error.what()));}
    session.reset(); // Python closes on its owning thread before Java refs die.
    FullLog(ANDROID_LOG_INFO, "QUEST_FULL_STAGE worker_stopped");
}

} // namespace quest_newton
