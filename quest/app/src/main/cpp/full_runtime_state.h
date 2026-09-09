#pragma once
#include "base_registration.h"
#include "control_boundary.h"
#include "environment_renderer.h"
#include "input_mapper.h"
#include "scene_snapshot.h"
#include "sim_menu.h"
#include "room_scene.h"
#include "object_interaction.h"
#include "scene_details.h"
#include "interaction_trace.h"
#include "remote_inspection.h"
#include "remote_scene.h"
#include "remote_scene_receiver.h"
#include "remote_scene_renderer.h"
#include "gimbal_client.h"
#include "gpu_timer.h"
#include <jni.h>
#include <chrono>
#include <deque>
#include <mutex>

namespace quest_newton {
// Every shared field is covered by mutex. The menu, raw input and registration
// below are exclusively owned by the XR thread; Python exclusively by worker.
struct FullRuntimeState {
    std::mutex mutex;
    FullInputFrame input;
    SimSettings requested_settings;
    std::uint64_t settings_generation = 1;
    std::deque<SimMenuCommand> commands;
    std::deque<ObjectEdit> object_edits;
    SceneDetails details;
    std::uint32_t selected_object=0;
    InputTime details_sampled{};
    std::uint64_t details_reference=0;
    bool details_have_contacts=false;
    std::string details_error;
    std::optional<kinematics::Pose> world_target;
    std::uint64_t target_reference=0;
    std::uint32_t target_generation=0;
    InputTime target_sampled{};
    double input_age_ms=-1,scene_age_ms=-1;
    bool show_diagnostics=false;
    InteractionTrace trace;
    SimMenuStatus status;
    bool fault = false;
    SceneMailbox scenes;
    std::shared_ptr<const RoomEnvironment> desired_room=std::make_shared<const RoomEnvironment>();
    bool room_ready = false;
    std::uint64_t room_applied_revision = 0;
    bool room_applied_enabled = false;
    std::string room_apply_error;

    JavaVM* vm = nullptr;
    jobject activity = nullptr;
    jclass bridge = nullptr;
    SimMenu menu;
    EnvironmentRenderer environment;
    RoomScene room_scene;
    RemoteSceneMailbox remote_frames;
    std::unique_ptr<RemoteSceneReceiver> remote_receiver;
    RemoteSceneRenderer remote_renderer;
    RemoteInspection remote_view;
    std::unique_ptr<GimbalControlClient> gimbal_client;
    GimbalAim gimbal_aim;
    unsigned gimbal_mode=0; // session-only; never restore an armed camera mode
    std::string gimbal_error;
    bool remote_inspecting=false;
    bool remote_frame_rendered=false,remote_timer_ready=false;
    verification::TimingRecorder remote_cpu_timing{1000000./90.},remote_gpu_timing{1000000./90.};
    verification::GpuTimer remote_gpu_timer{remote_gpu_timing};
    std::uint64_t remote_render_frames=0;
    std::chrono::steady_clock::time_point remote_stats_started{};
    std::uint64_t remote_aligned_stream=0,remote_aligned_identity=0;
    std::string remote_error;
    std::string remote_stats_json="null"; // published under mutex for worker evidence
    double last_remote_status=-1;
    bool room_api_available = false, room_permission_requested = false;
    bool room_permission_granted = false;
    bool room_refresh_pending = false;
    double room_permission_poll = -1;
    jmethodID room_permission_query = nullptr, room_permission_request = nullptr;
    BaseRegistration registration;
    SimSettings settings;
    InputValues raw{};
    kinematics::Pose head;
    OVR::Posef left_aim, right_aim;
    bool head_valid = false, left_aim_valid = false, right_aim_valid = false;
    bool menu_down = true, calibration_down = true, was_suppressed = true;
    bool fault_menu_shown = false, startup_menu_shown = false;
    std::uint64_t calibration_counter = 0, reference_counter = 0, suspension_counter = 0;
    std::optional<CalibrationEvent> calibration_event;
    double last_diagnostics_time = 0;
    ObjectPlacement object_placement;
    bool placement_release_pending=false;
    std::uint32_t next_object_id=1;
    std::uint64_t input_sequence=0;
    // Immutable benchmark placement after first tracked STAGE/head sample (XR owned).
    bool benchmark_placed = false;
    kinematics::Pose benchmark_stage_from_world, benchmark_world_from_base;
};
} // namespace quest_newton
