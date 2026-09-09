#include "interaction_trace.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>
#include <thread>

namespace allocation_fault {
std::atomic<std::size_t> minimum_size{std::numeric_limits<std::size_t>::max()};
std::atomic<bool> injected{false};
}
void* operator new(std::size_t size){
    auto minimum=allocation_fault::minimum_size.load();
    if(minimum!=std::numeric_limits<std::size_t>::max()&&size>=minimum&&
       allocation_fault::minimum_size.compare_exchange_strong(minimum,std::numeric_limits<std::size_t>::max())){
        allocation_fault::injected=true;throw std::bad_alloc{};
    }
    if(void* result=std::malloc(size?size:1))return result;
    throw std::bad_alloc{};
}
void operator delete(void* pointer)noexcept{std::free(pointer);}
void operator delete(void* pointer,std::size_t)noexcept{std::free(pointer);}

namespace {
using namespace quest_newton;
void Check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
using Bytes=std::vector<std::byte>;
Bytes Read(const std::filesystem::path& file){
    Bytes data(static_cast<std::size_t>(std::filesystem::file_size(file)));std::ifstream in(file,std::ios::binary);
    in.read(reinterpret_cast<char*>(data.data()),static_cast<std::streamsize>(data.size()));Check(in.good(),"read fixture");return data;
}
std::uint32_t U32(const Bytes& bytes,std::size_t at){
    std::uint32_t value=0;for(unsigned i=0;i<4;++i)value|=static_cast<std::uint32_t>(std::to_integer<unsigned char>(bytes[at+i]))<<(8*i);return value;
}
void Rehash(Bytes& bytes){
    std::uint64_t hash=14695981039346656037ULL;
    for(std::size_t i=0;i<bytes.size()-8;++i){hash^=std::to_integer<unsigned char>(bytes[i]);hash*=1099511628211ULL;}
    for(unsigned i=0;i<8;++i)bytes[bytes.size()-8+i]=static_cast<std::byte>((hash>>(8*i))&255);
}
Bytes LegacyFixture(){
    static constexpr unsigned char data[]={
#include "fixtures/qitr_v1_settings.inc"
    };
    Bytes result;for(auto value:data)result.push_back(static_cast<std::byte>(value));return result;
}
Bytes V2Fixture(){
    static constexpr unsigned char data[]={
#include "fixtures/qitr_v2_settings.inc"
    };
    Bytes result;for(auto value:data)result.push_back(static_cast<std::byte>(value));return result;
}
std::vector<std::size_t> SettingsOffsets(const Bytes& bytes){
    const std::string marker="version=1\n";std::vector<std::size_t> offsets;
    for(std::size_t at=0;at+marker.size()<=bytes.size();++at)
        if(std::equal(marker.begin(),marker.end(),bytes.begin()+at,
            [](char a,std::byte b){return static_cast<unsigned char>(a)==std::to_integer<unsigned char>(b);}))offsets.push_back(at);
    Check(offsets.size()==2,"fixture must contain header and input settings");return offsets;
}
std::size_t SettingsSize(const Bytes& bytes,std::size_t at){
    return std::to_integer<unsigned char>(bytes.at(at-2))+(static_cast<std::size_t>(std::to_integer<unsigned char>(bytes.at(at-1)))<<8);
}
std::string SettingsText(const Bytes& bytes,std::size_t which){
    const auto at=SettingsOffsets(bytes).at(which);
    return {reinterpret_cast<const char*>(bytes.data()+at),SettingsSize(bytes,at)};
}
void PutU32(Bytes& bytes,std::size_t at,std::uint32_t value){
    for(unsigned i=0;i<4;++i)bytes.at(at+i)=static_cast<std::byte>((value>>(8*i))&255);
}
Bytes RewriteSettings(Bytes bytes,std::size_t which,const std::string& text){
    const auto at=SettingsOffsets(bytes).at(which),old_size=SettingsSize(bytes,at);
    const auto header=U32(bytes,8);std::size_t record_at=header;
    if(at>=header){
        while(record_at<bytes.size()-40 && record_at+4+U32(bytes,record_at)<=at)record_at+=4+U32(bytes,record_at);
        Check(record_at<at&&at+old_size<=record_at+4+U32(bytes,record_at),"settings rewrite must remain inside its input record");
    }
    Check(text.size()<=4096,"test settings length exceeds contract");
    const auto delta=static_cast<std::ptrdiff_t>(text.size())-static_cast<std::ptrdiff_t>(old_size);
    Bytes replacement;for(unsigned char value:text)replacement.push_back(static_cast<std::byte>(value));
    bytes.erase(bytes.begin()+at,bytes.begin()+at+old_size);bytes.insert(bytes.begin()+at,replacement.begin(),replacement.end());
    bytes[at-2]=static_cast<std::byte>(text.size()&255);bytes[at-1]=static_cast<std::byte>((text.size()>>8)&255);
    const auto length_at=at<header?8:record_at;
    PutU32(bytes,length_at,static_cast<std::uint32_t>(static_cast<std::ptrdiff_t>(U32(bytes,length_at))+delta));
    Rehash(bytes);return bytes;
}
std::string ChangeSetting(std::string text,const std::string& key,const std::string& value){
    const auto at=text.find(key+"=");Check(at!=std::string::npos,"test setting was not serialized");
    text.replace(at,text.find('\n',at)-at+1,value.empty()?"":key+"="+value+"\n");return text;
}
struct Files {
    std::vector<std::filesystem::path> files;
    std::filesystem::path Next(){
        auto file=std::filesystem::temp_directory_path()/std::filesystem::path("quest-interaction-test-"+std::to_string(InteractionNowNs())+"-"+std::to_string(files.size())+".qitr");
        files.push_back(file);return file;
    }
    ~Files(){for(const auto& file:files){std::error_code error;std::filesystem::remove(file,error);}}
};
}
int main(){
    using namespace quest_newton;
    try{
        Files files;std::string error;InteractionTrace trace;SimSettings settings;
        const auto legacy=LegacyFixture();const auto old_replay=ReplayInteractionTrace(legacy);
        Check(legacy[4]==std::byte{1}&&old_replay.passed&&old_replay.control_count==1&&old_replay.input_count==1&&
              old_replay.run_id=="legacy_v1_synthetic","immutable old-writer Reset/Input fixture must replay");
        for(std::size_t which=0;which<2;++which){
            SimSettings old_settings;
            Check(ParseSettings(SettingsText(legacy,which),old_settings,error,SettingsParseMode::TraceV1)&&
                  old_settings.gripper_speed_mps==0&&!old_settings.gripper_force_hold&&old_settings.gripper_force_n==5,
                  "v1 header and input use legacy direct gripper settings");
            Check(old_settings.contact_profile==ContactProfile::OriginalMesh,"v1 implies Original contact model");
            Check(!ReplayInteractionTrace(RewriteSettings(legacy,which,SettingsText(legacy,which)+"contact_profile=legacy_mesh_v1\n")).passed,
                  "v1 header/input cannot introduce contact models");
            Check(!ReplayInteractionTrace(RewriteSettings(legacy,which,SettingsText(legacy,which)+"gripper_speed_mps=0\n")).passed,
                  "v1 must reject a rechecksummed new-only setting in either location");
        }
        const auto v2=V2Fixture();const auto v2_replay=ReplayInteractionTrace(v2);
        Check(v2[4]==std::byte{2}&&v2_replay.passed&&v2_replay.control_count==1&&v2_replay.input_count==1&&
              v2_replay.run_id=="legacy_v2_synthetic","immutable old v2 writer Reset/Input fixture must replay");
        for(std::size_t which=0;which<2;++which){
            SimSettings old_settings;
            Check(ParseSettings(SettingsText(v2,which),old_settings,error,SettingsParseMode::TraceV2)&&
                  old_settings.contact_profile==ContactProfile::OriginalMesh&&old_settings.gripper_force_hold&&
                  old_settings.gripper_speed_mps==.04&&old_settings.gripper_force_n==(which?9.125:7.),
                  "old v2 header/Input preserve gripper settings and imply Original model");
            Check(!ReplayInteractionTrace(RewriteSettings(v2,which,SettingsText(v2,which)+"contact_profile=legacy_mesh_v1\n")).passed,
                  "v2 header/input cannot introduce contact models");
            for(const auto key:{"gripper_force_hold","gripper_speed_mps","gripper_force_n"})
                Check(!ReplayInteractionTrace(RewriteSettings(v2,which,ChangeSetting(SettingsText(v2,which),key,""))).passed,
                      "legacy v2 still requires every gripper field");
        }
        settings.gripper_force_hold=true;settings.gripper_speed_mps=std::nextafter(.037,.2);
        settings.contact_profile=ContactProfile::FivePads;
        settings.gripper_force_n=std::nextafter(6.75,7.);
        Check(!trace.Start("../bad",settings,100,error),"unsafe recording nonce accepted");
        Check(trace.Start("control_A",settings,100,error),"start valid trace");
        Check(!trace.AppendInput({},99)&&!trace.Status().loss,"pre-start XR sample incorrectly poisons new capture");
        Check(!trace.Start("second",settings,100,error),"active trace replaced");
        std::int64_t now=100;FullControlMachine machine;
        machine.SetObserver([&](const auto& event){Check(trace.AppendControl(event,++now),"control append");});
        InteractionInputObservation raw;raw.head_valid=true;raw.input_sequence=3;raw.input.controller.trigger=std::numeric_limits<float>::quiet_NaN();
        raw.settings=settings;raw.settings.gripper_force_n=9.125;
        raw.settings.contact_profile=ContactProfile::PadsWithFriction;
        raw.input.calibration_event=CalibrationEvent{};
        Check(trace.AppendInput(raw,300),"raw observation accepted before worker's earlier timestamp");
        machine.Reset();ControllerSample sample;sample.focused=sample.stage_valid=sample.pose_active=sample.trigger_active=sample.calibrate_active=true;
        sample.position_valid=sample.orientation_valid=sample.position_tracked=sample.orientation_tracked=true;
        machine.Calibrate(sample,{},{});machine.Control(sample,{.7F,true},{},{},true);
        sample.trigger=1;sample.stage_from_grip.position[0]=.02F;
        machine.Control(sample,{.3F,true},{},{},true);machine.Suppress();machine.Control(sample,{0,true},{},{},true);machine.Invalidate();
        InteractionObservation observation;observation.kind=InteractionObservationKind::Clock;observation.values={.005,2,1,0};
        Check(trace.AppendObservation(observation,150),"out-of-order external clock observation accepted");
        trace.Stop(400);Check(trace.NeedsFlush()&&!trace.Recording()&&!trace.Status().loss,"stop marks complete trace pending");
        const auto file=files.Next();bool wrong_thread_saved=true;
        std::thread wrong([&]{std::string why;wrong_thread_saved=trace.Flush(file,why);});wrong.join();
        Check(!wrong_thread_saved&&!std::filesystem::exists(file),"nonworker Flush performed I/O");
        Check(trace.Flush(file,error),"save complete trace");
        auto bytes=Read(file);const auto replay=ReplayInteractionTrace(bytes);
        Check(bytes[4]==std::byte{3}&&bytes[5]==std::byte{0},"new captures must declare QITR v3 for explicit contact models");
        for(std::size_t which=0;which<2;++which){
            SimSettings captured;
            Check(ParseSettings(SettingsText(bytes,which),captured,error,SettingsParseMode::TraceV3)&&
                  captured==(which?raw.settings:settings),"v3 captures exact nondefault header/input gripper settings and models");
            for(const auto key:{"gripper_force_hold","gripper_speed_mps","gripper_force_n","contact_profile"})
                Check(!ReplayInteractionTrace(RewriteSettings(bytes,which,ChangeSetting(SettingsText(bytes,which),key,""))).passed,
                      "rechecksummed v3 settings cannot omit a required gripper/model field");
            for(const auto& [key,value]:{std::pair{"gripper_force_hold","2"},std::pair{"gripper_speed_mps","0"},
                                       std::pair{"gripper_force_n","nan"},std::pair{"gripper_force_n","21"},
                                       std::pair{"contact_profile","unknown"},std::pair{"contact_profile","2"}})
                Check(!ReplayInteractionTrace(RewriteSettings(bytes,which,ChangeSetting(SettingsText(bytes,which),key,value))).passed,
                      "rechecksummed v3 settings must validate models, types, ranges and force/speed combination");
        }
        Check(replay.passed&&replay.control_count==7&&replay.input_count==1&&replay.external_observations==1,"exact native control replay failed");
        auto corrupt=bytes;corrupt[corrupt.size()/2]^=std::byte{1};Check(!ReplayInteractionTrace(corrupt).passed,"checksum corruption accepted");
        corrupt=bytes;corrupt.pop_back();Check(!ReplayInteractionTrace(corrupt).passed,"truncated footer accepted");
        corrupt=bytes;const auto header=U32(corrupt,8);corrupt[header+8]=std::byte{9};Rehash(corrupt);
        Check(!ReplayInteractionTrace(corrupt).passed,"rechecksummed missing sequence accepted");
        corrupt=bytes;corrupt[header+4]=std::byte{9};Rehash(corrupt);Check(!ReplayInteractionTrace(corrupt).passed,"unknown record type accepted");
        corrupt=bytes;corrupt[4]=std::byte{4};Rehash(corrupt);Check(!ReplayInteractionTrace(corrupt).passed,"unknown trace version accepted");
        corrupt=bytes;corrupt[4]=std::byte{2};Rehash(corrupt);Check(!ReplayInteractionTrace(corrupt).passed,"v3 model accepted under a forged v2 header");
        corrupt=bytes;corrupt[4]=std::byte{1};Rehash(corrupt);Check(!ReplayInteractionTrace(corrupt).passed,"v2 fields accepted under a forged v1 header");

        InteractionTrace mismatch;Check(mismatch.Start("wrong_expected",settings,0,error),"mismatch start");
        FullControlEvent reset;reset.expected.targets[0]=.1F;
        Check(mismatch.AppendControl(reset,1),"syntactically valid mismatch append");mismatch.Stop(2);
        const auto wrong_file=files.Next();Check(mismatch.Flush(wrong_file,error),"mismatch save");
        Check(!ReplayInteractionFile(wrong_file).passed,"expected outputs were trusted rather than replayed");

        InteractionTrace lost;Check(lost.Start("lost_seed",settings,0,error),"loss start");
        FullControlEvent no_seed;no_seed.operation=FullControlOperation::Control;
        Check(!lost.AppendControl(no_seed,1)&&lost.Status().loss,"missing reset seed did not fail closed");
        const auto loss_file=files.Next();Check(lost.Flush(loss_file,error),"loss evidence save");
        Check(!ReplayInteractionFile(loss_file).passed,"loss recording accepted for replay");

        InteractionTrace cap;Check(cap.Start("event_cap",settings,0,error),"cap start");
        FullControlMachine seed;FullControlEvent seed_event;seed.SetObserver([&](const auto& event){seed_event=event;});seed.Reset();
        Check(cap.AppendControl(seed_event,0),"cap seed");
        for(std::uint32_t i=1;i<InteractionTrace::kMaxEvents;++i)Check(cap.AppendObservation({},i),"event capacity stopped too early");
        Check(!cap.Recording()&&cap.Status().event_count==40000&&!cap.Status().loss,"event cap does not stop cleanly at boundary");
        Check(!cap.AppendObservation({},40000),"event cap overflow accepted");
        const auto cap_file=files.Next();Check(cap.Flush(cap_file,error),"cap save");
        Check(ReplayInteractionFile(cap_file).passed&&std::filesystem::file_size(cap_file)<=InteractionTrace::kMaxBytes,"bounded complete prefix replay failed");

        InteractionTrace duration;Check(duration.Start("duration",settings,0,error),"duration start");duration.AppendControl(seed_event,0);
        Check(!duration.AppendObservation({},InteractionTrace::kMaxDurationNs)&&duration.Status().stop_reason==InteractionStopReason::DurationLimit,"60-second limit not enforced");
        const auto duration_file=files.Next();Check(duration.Flush(duration_file,error)&&ReplayInteractionFile(duration_file).passed,"duration-limited trace invalid");

        InteractionTrace preparation_failure;
        // Use a bounded heap-backed identity so the injected failure targets
        // save preparation, not small Debug CRT iterator bookkeeping.
        const std::string long_id(64,'a');
        Check(preparation_failure.Start(long_id,settings,0,error),"allocation-failure trace start");
        Check(preparation_failure.AppendControl(seed_event,1),"allocation-failure reset seed");
        preparation_failure.Stop(2);const auto allocation_file=files.Next();
        bool saved=false,escaped=false;
        allocation_fault::injected=false;allocation_fault::minimum_size=long_id.size()+1;
        try{saved=preparation_failure.Flush(allocation_file,error);}catch(...){escaped=true;}
        allocation_fault::minimum_size=std::numeric_limits<std::size_t>::max();
        Check(allocation_fault::injected,"save preparation allocation was not exercised");
        Check(!escaped&&!saved,"save preparation allocation failure must return a retryable failure");
        const auto retained=preparation_failure.Status();
        Check(preparation_failure.NeedsFlush()&&retained.pending_flush&&!retained.saved&&!retained.loss&&
              retained.run_id==long_id&&retained.control_count==1,
              "preparation failure lost or wedged the pending recording");
        Check(!preparation_failure.Start("replacement",settings,3,error),"failed save allowed unsaved data to be replaced");
        Check(preparation_failure.Flush(allocation_file,error),"save retry after preparation failure failed");
        const auto recovered=ReplayInteractionFile(allocation_file);
        Check(recovered.passed&&recovered.control_count==1&&recovered.run_id==long_id,
              "preparation retry did not preserve exact replayable recording data");

        InteractionTrace write_failure;Check(write_failure.Start("write_retry",settings,0,error),"write-failure trace start");
        Check(write_failure.AppendControl(seed_event,1),"write-failure reset seed");write_failure.Stop(2);
        const auto missing_parent=files.Next();
        Check(!write_failure.Flush(missing_parent/"trace.qitr",error)&&write_failure.NeedsFlush(),
              "filesystem failure did not preserve a pending recording");
        const auto write_retry_file=files.Next();
        Check(write_failure.Flush(write_retry_file,error)&&ReplayInteractionFile(write_retry_file).passed,
              "filesystem retry duplicated or lost trace framing");

        InteractionTrace inactive;Check(inactive.Start("faulted_no_events",settings,100,error),"inactive start");inactive.AppendControl(seed_event,100);
        inactive.Expire(100+InteractionTrace::kMaxDurationNs-1);
        Check(inactive.Recording()&&!inactive.NeedsFlush(),"cooperative expiry ended capture early");
        // Simulate a faulted worker/no XR frames: nothing is appended after seed.
        inactive.Expire(100+InteractionTrace::kMaxDurationNs);
        Check(!inactive.Recording()&&inactive.NeedsFlush()&&!inactive.Status().loss&&inactive.Status().event_count==1&&
              inactive.Status().stop_reason==InteractionStopReason::DurationLimit,"no-append capture did not expire");
        inactive.Expire(100+InteractionTrace::kMaxDurationNs+1000);
        const auto inactive_file=files.Next();
        Check(inactive.Flush(inactive_file,error)&&ReplayInteractionFile(inactive_file).passed,"expired faulted-worker prefix is not replayable");

        InteractionTrace concurrent;Check(concurrent.Start("concurrent",settings,0,error),"concurrent start");concurrent.AppendControl(seed_event,0);
        std::atomic<bool> accepted=true;
        std::thread producer([&]{for(int i=0;i<100;++i)if(!concurrent.AppendInput(raw,1000+i))accepted=false;});
        for(int i=0;i<100;++i)if(!concurrent.AppendObservation({},100+i))accepted=false;
        producer.join();concurrent.Stop(2000);const auto concurrent_file=files.Next();
        Check(accepted&&concurrent.Flush(concurrent_file,error)&&ReplayInteractionFile(concurrent_file).passed,"concurrent producer clocks/sequence invalid");
        std::cout<<"interaction trace replay, framing/loss/capacity, worker save and concurrent clocks passed\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
