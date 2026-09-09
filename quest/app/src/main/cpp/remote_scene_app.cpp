#include "quest_newton_app.h"
#include "full_runtime_state.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace quest_newton {
namespace {
InspectionIdentity ViewKey(const RemoteSceneFrame& frame){return {frame.source_id,frame.world_epoch,frame.calibration_id,static_cast<std::uint64_t>(frame.representation),frame.map_generation};}
const char* GeometryName(RemoteSceneRepresentation representation){
    switch(representation){
    case RemoteSceneRepresentation::Points:return "points";
    case RemoteSceneRepresentation::Prepared:return "robot-built surfaces";
    case RemoteSceneRepresentation::Unprepared:return "Quest-built surfaces";
    }
    return "unknown";
}
bool ViewMatches(const FullRuntimeState& state,const RemoteSceneSnapshot& snapshot){return snapshot.frame &&
    state.remote_view.Matches(ViewKey(*snapshot.frame),state.reference_counter) &&
    state.remote_aligned_stream==snapshot.stream_generation && state.remote_aligned_identity==snapshot.identity_revision;}
bool IsRemoteCommand(SimMenuCommand command){return command>=SimMenuCommand::RemoteDemo && command<=SimMenuCommand::GimbalResetMap;}
}
void QuestNewtonApp::InitRemoteScene(const xrJava* context){
    auto& f=*full_;
    f.remote_receiver=std::make_unique<RemoteSceneReceiver>(f.remote_frames);
    if(!f.remote_receiver->Init(context->Vm,context->Env,context->ActivityObject,f.remote_error))
        f.status.remote="Remote receiver unavailable: "+f.remote_error;
    f.gimbal_client=std::make_unique<GimbalControlClient>();
    f.gimbal_client->Init(context->Vm,context->Env,context->ActivityObject,f.gimbal_error);
}
void QuestNewtonApp::InitRemoteSceneSession(){
    auto& f=*full_;
    if(!f.remote_renderer.Init())f.remote_error="Remote renderer unavailable: "+f.remote_renderer.Error();
    ResetRemoteTiming();
    f.remote_view.Invalidate();f.last_remote_status=-1;
}
void QuestNewtonApp::StopRemoteSceneSession(const xrJava* context){
    auto& f=*full_;
    if(f.remote_receiver)f.remote_receiver->Stop(context->Env);
    if(f.gimbal_client)f.gimbal_client->Stop(context->Env);
    f.gimbal_aim.Reset();f.gimbal_mode=0;
    f.remote_gpu_timer.Shutdown();f.remote_timer_ready=false;
    f.remote_view.Invalidate();f.remote_renderer.Shutdown();f.last_remote_status=-1;
}
void QuestNewtonApp::ResetRemoteTiming(){
    auto& f=*full_;
    f.remote_gpu_timer.Shutdown();f.remote_cpu_timing.Reset();f.remote_gpu_timing.Reset();
    f.remote_render_frames=0;f.remote_stats_started=std::chrono::steady_clock::now();
    f.remote_timer_ready=f.remote_gpu_timer.Init();
}
bool QuestNewtonApp::HandleRemoteCommand(SimMenuCommand command){
    auto& f=*full_;
    if(!IsRemoteCommand(command)){
        if(f.remote_inspecting && (command==SimMenuCommand::PlaceCube || command==SimMenuCommand::MoveCube ||
            command==SimMenuCommand::ResetCube || command==SimMenuCommand::DeleteCube || command==SimMenuCommand::SpawnBox ||
            command==SimMenuCommand::RemoveBox || command==SimMenuCommand::Recenter)){
            f.remote_error="Exit inspection to edit local simulation objects or placement";f.last_remote_status=-1;return true;
        }
        return false;
    }
    const auto* context=GetContext();
    try {
        if(command>=SimMenuCommand::GimbalConnect){
            f.gimbal_aim.Reset();
            if(f.gimbal_client)f.gimbal_client->Submit({GimbalOperation::Hold,false,0,0,std::chrono::steady_clock::now()});
            if(command==SimMenuCommand::GimbalConnect){
                f.gimbal_mode=0;f.gimbal_error.clear();
                if(!f.gimbal_client)f.gimbal_error="Camera client unavailable";
                else f.gimbal_client->Start(context->Env,f.gimbal_error);
            }else if(command==SimMenuCommand::GimbalDisconnect){
                if(f.gimbal_client)f.gimbal_client->Stop(context->Env);
                f.gimbal_mode=0;f.gimbal_error.clear();
            }else if(command==SimMenuCommand::GimbalMode){
                f.gimbal_mode=(f.gimbal_mode+1)%3;f.gimbal_error.clear();
            }else{
                const auto scene=f.remote_frames.Read();
                const auto control=f.gimbal_client?f.gimbal_client->Snapshot():GimbalClientSnapshot{};
                if(!scene.frame || !control.connected || control.feedback.source_id!=scene.frame->source_id ||
                   !RemoteSceneFresh(scene,std::chrono::steady_clock::now()) || !f.gimbal_client->RequestMapReset())
                    f.gimbal_error="Connect matching scene and camera control before clearing the map";
                else f.gimbal_error="Map clear requested; Align view after the new map arrives";
            }
            f.last_remote_status=-1;return true;
        }
        const bool rgb=command==SimMenuCommand::RemoteDemoRgbPrepared || command==SimMenuCommand::RemoteDemoRgbUnprepared;
        const bool demo=command==SimMenuCommand::RemoteDemo || command==SimMenuCommand::RemoteDemoPrepared || command==SimMenuCommand::RemoteDemoUnprepared || rgb;
        f.gimbal_aim.Reset();
        if(f.gimbal_client)f.gimbal_client->Submit({GimbalOperation::Hold,false,0,0,std::chrono::steady_clock::now()});
        if(demo || command==SimMenuCommand::RemoteConnect){
            if(f.gimbal_client)f.gimbal_client->Stop(context->Env);
            f.gimbal_mode=0;
            f.remote_inspecting=false;f.remote_view.Invalidate();SuspendFullInput(false);
            f.remote_error.clear();
            if(!f.remote_receiver){f.remote_error="Remote receiver unavailable";return true;}
            f.remote_receiver->Stop(context->Env);
            const auto representation=command==SimMenuCommand::RemoteDemoPrepared || command==SimMenuCommand::RemoteDemoRgbPrepared?RemoteSceneRepresentation::Prepared:
                command==SimMenuCommand::RemoteDemoUnprepared || command==SimMenuCommand::RemoteDemoRgbUnprepared?RemoteSceneRepresentation::Unprepared:RemoteSceneRepresentation::Points;
            const bool started=demo?f.remote_receiver->StartDemo(context->Env,f.remote_error,representation,rgb):
                f.remote_receiver->Start(context->Env,f.remote_error);
            if(!started && f.remote_error.empty())f.remote_error="Remote source could not start";
        }else if(command==SimMenuCommand::RemoteDisconnect){
            if(f.remote_receiver)f.remote_receiver->Stop(context->Env);
            if(f.gimbal_client)f.gimbal_client->Stop(context->Env);
            f.gimbal_mode=0;
            f.remote_error.clear();
        }else if(command==SimMenuCommand::RemoteInspect && f.remote_inspecting){
            f.remote_inspecting=false;f.remote_error.clear();SuspendFullInput(false);
        }else{
            const auto snapshot=f.remote_frames.Read();
            if(!snapshot.frame || (!snapshot.frame->HasGeometry() && !(snapshot.frame->wire_version==3 && (snapshot.frame->map_flags&1))))f.remote_error="No complete remote view; load Demo or Connect first";
            else if(!f.head_valid || !pending_input_.focused || !pending_input_.stage_valid)f.remote_error="Tracked head and STAGE are required for inspection";
            else if(!f.remote_renderer.Ready())f.remote_error="Remote renderer is unavailable; local simulation remains available";
            else {
                const auto& frame=*snapshot.frame;const auto key=ViewKey(frame);
                if(command==SimMenuCommand::RemoteInspect || command==SimMenuCommand::RemoteAlign){
                    if(f.remote_view.Align(key,frame.suggested_observer,f.head,f.reference_counter)){
                        f.remote_aligned_stream=snapshot.stream_generation;f.remote_aligned_identity=snapshot.identity_revision;
                        ResetRemoteTiming();
                        f.remote_inspecting=true;f.remote_error.clear();SuspendFullInput(false);
                    }else f.remote_error="Cannot align this view; look level and retry";
                }else if(!f.remote_inspecting || !ViewMatches(f,snapshot)){
                    f.remote_error="Enter and align inspection before moving the view";
                }else if(command==SimMenuCommand::RemoteScale){
                    const auto old=f.remote_view.Scale();const float scale=old<.5F?.5F:old<1.F?1.F:old<2.F?2.F:.25F;
                    if(!f.remote_view.SetScale(scale,f.head))f.remote_error="View scale would exceed map bounds";
                    else f.remote_error.clear();
                }else{
                    if(!f.remote_view.Step(f.head,command==SimMenuCommand::RemoteForward?.5F:-.5F))f.remote_error="View step would exceed map bounds";
                    else f.remote_error.clear();
                }
            }
        }
    }catch(const std::exception& error){f.remote_error=std::string("Remote view: ")+error.what();}
    f.last_remote_status=-1;return true;
}
void QuestNewtonApp::UpdateRemoteScene(){
    auto& f=*full_;
    const auto now=std::chrono::steady_clock::now();
    const double seconds=std::chrono::duration<double>(now.time_since_epoch()).count();
    if(f.last_remote_status>=0 && seconds-f.last_remote_status<.5)return;
    f.last_remote_status=seconds;
    const auto snapshot=f.remote_frames.Read();std::ostringstream message;
    if(f.remote_receiver)message<<f.remote_receiver->Status();else message<<"Remote receiver unavailable";
    if(snapshot.frame){
        const auto& frame=*snapshot.frame;
        message<<" | "<<((frame.flags&kRemoteSceneSynthetic)?"SYNTHETIC":(frame.flags&kRemoteSceneRecorded)?"RECORDED":"LIVE")
            <<" "<<std::popcount(frame.contributing_mask)<<'/'<<std::popcount(frame.expected_camera_mask)<<" cameras, "<<GeometryName(frame.representation);
        if(frame.representation==RemoteSceneRepresentation::Points)message<<" "<<frame.points.size();
        else message<<" "<<frame.indices.size()/3<<" triangles, "<<frame.atlas_width<<'x'<<frame.atlas_height<<" texture";
        if(frame.wire_version==3){
            const auto rgb=std::count_if(frame.observations.begin(),frame.observations.end(),[](const auto& o){return o.image_format==3 && (o.flags&2);});
            message<<" | "<<rgb<<" RGB views, "<<frame.retained_view_count<<" retained";
            if(frame.oldest_observation_ns){
                const double elapsed=(frame.flags&(kRemoteSceneSynthetic|kRemoteSceneRecorded))?0.:RemoteSceneAgeMs(snapshot,now);
                message<<", oldest "<<std::fixed<<std::setprecision(1)<<(double(frame.produced_ns-frame.oldest_observation_ns)/1e9+elapsed/1000.)<<" s";
            }
            if(!(frame.map_flags&1))message<<" | Camera tracking lost";
        }
        if(frame.flags&kRemoteSceneTruncated)message<<" (resolution capped)";
        if(!(frame.flags&(kRemoteSceneSynthetic|kRemoteSceneRecorded)) && !RemoteSceneFresh(snapshot,now))
            message<<" | FROZEN "<<std::fixed<<std::setprecision(0)<<RemoteSceneAgeMs(snapshot,now)<<" ms since receive";
        if(f.remote_inspecting && !ViewMatches(f,snapshot)){f.remote_view.Invalidate();message<<" | Map/reference changed: Align view";}
    }
    if(!f.remote_error.empty())message<<" | "<<f.remote_error;
    const auto cpu=f.remote_cpu_timing.Snapshot(),gpu=f.remote_gpu_timing.Snapshot();
    std::ostringstream stats;stats<<std::setprecision(17)<<std::boolalpha;
    stats<<"{\"scope\":\"inspection_since_alignment\",\"active\":"<<f.remote_inspecting
        <<",\"alignment_current\":"<<ViewMatches(f,snapshot)<<",\"frames\":"<<f.remote_render_frames
        <<",\"elapsed_s\":"<<std::chrono::duration<double>(now-f.remote_stats_started).count()
        <<",\"cpu_samples\":"<<cpu.count<<",\"cpu_mean_us\":"<<cpu.mean_us<<",\"cpu_p99_us\":"<<cpu.p99_us
        <<",\"gpu_samples\":"<<gpu.count<<",\"gpu_mean_us\":"<<gpu.mean_us<<",\"gpu_p99_us\":"<<gpu.p99_us
        <<",\"gpu_supported\":"<<(f.remote_timer_ready && f.remote_gpu_timer.Supported())
        <<",\"lifetime_gpu_disjoints\":"<<f.remote_gpu_timer.DisjointCount()<<",\"point_count\":"<<(snapshot.frame?snapshot.frame->points.size():0)
        <<",\"representation\":"<<(snapshot.frame?static_cast<unsigned>(snapshot.frame->representation):0)
        <<",\"vertex_count\":"<<(snapshot.frame?snapshot.frame->vertices.size()+snapshot.frame->rgb_vertices.size():0)
        <<",\"map_generation\":"<<(snapshot.frame?snapshot.frame->map_generation:0)
        <<",\"retained_views\":"<<(snapshot.frame?snapshot.frame->retained_view_count:0)
        <<",\"triangle_count\":"<<(snapshot.frame?snapshot.frame->indices.size()/3:0)
        <<",\"atlas_width\":"<<(snapshot.frame?snapshot.frame->atlas_width:0)
        <<",\"atlas_height\":"<<(snapshot.frame?snapshot.frame->atlas_height:0)<<'}';
    const std::lock_guard lock(f.mutex);
    f.status.remote=message.str();f.status.remote_inspection=f.remote_inspecting;f.status.remote_scale=f.remote_view.Scale();
    f.remote_stats_json=stats.str();
    if(f.remote_inspecting){
        f.status.gpu="Inspection GPU "+(gpu.available?std::to_string(gpu.mean_us/1000.)+" ms":std::string("pending/unavailable"))+
            " | XR CPU "+(cpu.available?std::to_string(cpu.mean_us/1000.)+" ms":std::string("pending"));
    }
}
void QuestNewtonApp::UpdateGimbalControl(){
    auto& f=*full_;
    const auto now=std::chrono::steady_clock::now();
    const auto control=f.gimbal_client?f.gimbal_client->Snapshot():GimbalClientSnapshot{};
    const auto scene=f.remote_frames.Read();
    const bool feedback_fresh=control.connected && control.feedback.received!=GimbalTime{} &&
        now>=control.feedback.received && now-control.feedback.received<=std::chrono::milliseconds(150);
    const bool paired=scene.frame && feedback_fresh && control.feedback.source_id==scene.frame->source_id &&
        control.feedback.map_generation==scene.frame->map_generation;
    const bool scene_ready=paired && f.remote_receiver && f.remote_receiver->Running() &&
        RemoteSceneFresh(scene,now) && scene.frame->wire_version==3 && (scene.frame->map_flags&1) && ViewMatches(f,scene);
    GimbalAimInput input;input.now=now;input.feedback=control.feedback;
    if(scene.frame)input.epoch={scene.frame->source_id,scene.frame->world_epoch,scene.frame->map_generation,
                               f.reference_counter,control.feedback.session};
    input.eligible=f.gimbal_mode!=0 && f.remote_inspecting && !f.menu.IsOpen() && pending_input_.focused &&
        pending_input_.stage_valid && f.head_valid && scene_ready && (control.feedback.flags&4) &&
        (f.gimbal_mode==1 || f.right_aim_valid);
    const auto& clutch=f.raw[static_cast<std::size_t>(InputId::LeftTrigger)];
    input.clutch_active=clutch.active;input.clutch=clutch.value;
    if(f.gimbal_mode==1 && f.head_valid)input.orientation=f.head;
    else if(f.right_aim_valid)input.orientation={{f.right_aim.Translation.x,f.right_aim.Translation.y,f.right_aim.Translation.z},
        {f.right_aim.Rotation.x,f.right_aim.Rotation.y,f.right_aim.Rotation.z,f.right_aim.Rotation.w}};
    const auto intent=f.gimbal_aim.Update(input);
    if(f.gimbal_client)f.gimbal_client->Submit(intent);
    std::ostringstream message;
    if(control.connected){
        message<<"SIM camera "<<std::fixed<<std::setprecision(0)<<control.feedback.pan*57.2957795F<<" / "<<control.feedback.tilt*57.2957795F<<" deg. ";
        if(!feedback_fresh)message<<"Feedback stale; holding.";
        else if(!paired)message<<"Connect the matching remote scene.";
        else if(!scene_ready || !f.remote_inspecting)message<<"Enter inspection and Align view.";
        else message<<"Close menu; release then hold LEFT trigger to aim.";
    }else message<<(control.running?"Connecting simulated camera...":"Camera control disconnected; simulation only.");
    if(!control.error.empty())message<<" "<<control.error;
    if(!f.gimbal_error.empty())message<<" "<<f.gimbal_error;
    const std::lock_guard lock(f.mutex);f.status.camera=message.str();f.status.camera_mode=f.gimbal_mode;
}
bool QuestNewtonApp::RenderRemoteScene(const OVRFW::ovrApplFrameIn& in,OVRFW::ovrRendererOutput& out){
    auto& f=*full_;if(!f.remote_inspecting)return false;
    const auto snapshot=f.remote_frames.Read();
    if(snapshot.frame && !ViewMatches(f,snapshot))f.remote_view.Invalidate();
    if(snapshot.frame && f.head_valid && pending_input_.stage_valid &&
       ViewMatches(f,snapshot) && f.remote_renderer.Ready()){
        if(f.remote_renderer.Upload(snapshot)){
            const bool stale=!(snapshot.frame->flags&(kRemoteSceneSynthetic|kRemoteSceneRecorded)) &&
                !RemoteSceneFresh(snapshot,std::chrono::steady_clock::now());
            const auto* framebuffer=GetFrameBuffer(0);
            f.remote_frame_rendered=f.remote_renderer.Append(f.remote_view.StageFromMap(),static_cast<float>(framebuffer->Height),stale,true,out.Surfaces);
        }else{f.remote_error="Remote upload failed: "+f.remote_renderer.Error();f.last_remote_status=-1;}
    }
    f.menu.Render(in,out);rendered_body_count_=0;return true;
}
} // namespace quest_newton
