#include "interaction_trace.h"
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace quest_newton {
namespace {
using Bytes=std::vector<std::byte>;
constexpr std::size_t kFooterBytes=40;
constexpr unsigned char kControl=1,kInput=2,kObservation=3;
struct Writer {
    Bytes bytes;
    void U(std::uint64_t value,unsigned n){for(unsigned i=0;i<n;++i)bytes.push_back(static_cast<std::byte>((value>>(8*i))&255));}
    void I(std::int64_t value){U(std::bit_cast<std::uint64_t>(value),8);}
    void F(float value){U(std::bit_cast<std::uint32_t>(value),4);}
    void D(double value){U(std::bit_cast<std::uint64_t>(value),8);}
    void B(bool value){U(value?1:0,1);}
    void Magic(const char* value){for(unsigned i=0;i<4;++i)U(static_cast<unsigned char>(value[i]),1);}
    void Text(std::string_view value,std::size_t limit){
        if(value.size()>limit)throw std::runtime_error("trace string exceeds bound");
        U(value.size(),2);for(unsigned char c:value)U(c,1);
    }
    void Pose(const kinematics::Pose& value){for(float f:value.position)F(f);for(float f:value.rotation)F(f);}
};
struct Reader {
    std::span<const std::byte> bytes;
    std::size_t at=0;
    std::uint64_t U(unsigned n){
        if(n>8||at>bytes.size()||n>bytes.size()-at)throw std::runtime_error("truncated trace field");
        std::uint64_t value=0;for(unsigned i=0;i<n;++i)value|=static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[at++]))<<(8*i);
        return value;
    }
    std::int64_t I(){return std::bit_cast<std::int64_t>(U(8));}
    float F(){return std::bit_cast<float>(static_cast<std::uint32_t>(U(4)));}
    double D(){return std::bit_cast<double>(U(8));}
    bool B(){const auto value=U(1);if(value>1)throw std::runtime_error("invalid trace boolean");return value!=0;}
    void Magic(const char* expected){for(unsigned i=0;i<4;++i)if(U(1)!=static_cast<unsigned char>(expected[i]))throw std::runtime_error("invalid trace magic");}
    std::string Text(std::size_t limit){
        const auto length=static_cast<std::size_t>(U(2));
        if(length>limit||length>bytes.size()-at)throw std::runtime_error("invalid trace string length");
        std::string value;value.reserve(length);for(std::size_t i=0;i<length;++i)value.push_back(static_cast<char>(U(1)));return value;
    }
    kinematics::Pose Pose(){kinematics::Pose value;for(auto& f:value.position)f=F();for(auto& f:value.rotation)f=F();return value;}
    bool Done()const{return at==bytes.size();}
};
bool RunId(std::string_view value){
    return !value.empty()&&value.size()<=64&&std::all_of(value.begin(),value.end(),[](char c){return
        (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-';});
}
std::uint64_t Checksum(std::span<const std::byte> bytes){
    std::uint64_t value=14695981039346656037ULL;
    for(auto b:bytes){value^=std::to_integer<unsigned char>(b);value*=1099511628211ULL;}return value;
}
std::int64_t Ticks(InputTime value){return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();}
InputTime Time(std::int64_t value){return InputTime(std::chrono::duration_cast<InputTime::duration>(std::chrono::nanoseconds(value)));}
void Sample(Writer& w,const ControllerSample& s){
    w.Pose(s.stage_from_grip);w.F(s.trigger);
    const std::array<bool,10> flags{s.focused,s.stage_valid,s.pose_active,s.trigger_active,s.calibrate_active,
        s.position_valid,s.orientation_valid,s.position_tracked,s.orientation_tracked,s.calibrate_pressed};
    unsigned bits=0;for(unsigned i=0;i<flags.size();++i)if(flags[i])bits|=1U<<i;w.U(bits,2);
}
ControllerSample Sample(Reader& r){
    ControllerSample s;s.stage_from_grip=r.Pose();s.trigger=r.F();const auto bits=r.U(2);
    if(bits&~1023ULL)throw std::runtime_error("unknown controller flags");
    const std::array<bool*,10> flags{&s.focused,&s.stage_valid,&s.pose_active,&s.trigger_active,&s.calibrate_active,
        &s.position_valid,&s.orientation_valid,&s.position_tracked,&s.orientation_tracked,&s.calibrate_pressed};
    for(unsigned i=0;i<flags.size();++i)*flags[i]=(bits&(1ULL<<i))!=0;return s;
}
void Value(Writer& w,InputValue value){w.F(value.value);w.B(value.active);}
InputValue Value(Reader& r){const auto value=r.F();return {value,r.B()};}
void Calibration(Writer& w,const CalibrationEvent& e){
    Sample(w,e.controller);w.Pose(e.stage_from_world);w.Pose(e.world_from_base);w.I(Ticks(e.sampled));
    w.U(e.settings_generation,8);w.U(e.reference,8);w.U(e.suspension,8);
}
CalibrationEvent Calibration(Reader& r){
    CalibrationEvent e;e.controller=Sample(r);e.stage_from_world=r.Pose();e.world_from_base=r.Pose();e.sampled=Time(r.I());
    e.settings_generation=r.U(8);e.reference=r.U(8);e.suspension=r.U(8);return e;
}
void Frame(Writer& w,const FullInputFrame& f){
    Sample(w,f.controller);for(auto value:f.actions)Value(w,value);
    w.Pose(f.stage_from_world);w.Pose(f.world_from_base);w.B(f.registered);w.B(f.menu_open);
    w.U(f.calibration,8);w.U(f.reference,8);w.U(f.suspension,8);w.I(Ticks(f.sampled));
    w.B(f.calibration_event.has_value());if(f.calibration_event)Calibration(w,*f.calibration_event);
}
FullInputFrame Frame(Reader& r){
    FullInputFrame f;f.controller=Sample(r);for(auto& value:f.actions)value=Value(r);
    f.stage_from_world=r.Pose();f.world_from_base=r.Pose();f.registered=r.B();f.menu_open=r.B();
    f.calibration=r.U(8);f.reference=r.U(8);f.suspension=r.U(8);f.sampled=Time(r.I());
    if(r.B())f.calibration_event=Calibration(r);return f;
}
void Settings(Writer& w,const SimSettings& settings){
    std::string error;if(!ValidateSettings(settings,error))throw std::runtime_error("invalid recorded settings: "+error);
    w.Text(SerializeSettings(settings),4096);
}
SimSettings Settings(Reader& r,std::uint64_t trace_version){
    SimSettings value;std::string error;
    const auto mode=trace_version==1?SettingsParseMode::TraceV1:trace_version==2?SettingsParseMode::TraceV2:SettingsParseMode::TraceV3;
    if(!ParseSettings(r.Text(4096),value,error,mode))throw std::runtime_error("invalid recorded settings: "+error);return value;
}
void ControlArguments(Writer& w,const FullControlEvent& e){
    const auto op=static_cast<unsigned char>(e.operation);if(op>4)throw std::runtime_error("invalid control operation");
    w.U(op,1);Sample(w,e.sample);Value(w,e.gripper);w.Pose(e.stage_from_base);w.Pose(e.palm_offset);w.B(e.allowed);
}
FullControlEvent ControlArguments(Reader& r){
    FullControlEvent e;const auto op=r.U(1);if(op>4)throw std::runtime_error("invalid control operation");
    e.operation=static_cast<FullControlOperation>(op);e.sample=Sample(r);e.gripper=Value(r);
    e.stage_from_base=r.Pose();e.palm_offset=r.Pose();e.allowed=r.B();return e;
}
void Output(Writer& w,const FullControlOutput& o){
    for(float f:o.targets)w.F(f);w.F(o.gripper);w.B(o.input_allowed);w.B(o.gripper_rearmed);
    for(float f:o.mapped.joints)w.F(f);w.Pose(o.mapped.robot_base_from_stage);w.Pose(o.mapped.robot_base_from_grip);
    w.B(o.mapped.calibrated);w.B(o.mapped.engaged);w.Text(o.mapped.reason,128);
}
void Apply(FullControlMachine& machine,const FullControlEvent& event){
    switch(event.operation){
    case FullControlOperation::Reset:machine.Reset();break;
    case FullControlOperation::Invalidate:machine.Invalidate();break;
    case FullControlOperation::Suppress:machine.Suppress();break;
    case FullControlOperation::Calibrate:machine.Calibrate(event.sample,event.stage_from_base,event.palm_offset);break;
    case FullControlOperation::Control:machine.Control(event.sample,event.gripper,event.stage_from_base,event.palm_offset,event.allowed);break;
    }
}
} // namespace

std::int64_t InteractionNowNs(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
bool InteractionTrace::Start(std::string run_id,const SimSettings& settings,std::int64_t now_ns,std::string& error){
    const std::lock_guard lock(mutex_);
    if(status_.recording||status_.pending_flush||flushing_){error="Stop and save the preceding recording first";return false;}
    if(!RunId(run_id)||now_ns<0||now_ns>std::numeric_limits<std::int64_t>::max()-kMaxDurationNs){error="Invalid recording ID or start time";return false;}
    try{
        Writer w;w.Magic("QITR");w.U(3,2);w.U(0,2);w.U(0,4);w.I(now_ns);w.Text(run_id,64);Settings(w,settings);
        const auto size=static_cast<std::uint32_t>(w.bytes.size());
        for(unsigned i=0;i<4;++i)w.bytes[8+i]=static_cast<std::byte>((size>>(8*i))&255);
        w.bytes.reserve(kMaxBytes);data_=std::move(w.bytes);status_={};status_.recording=true;status_.run_id=std::move(run_id);
        start_ns_=last_ns_=end_ns_=now_ns;seeded_=false;worker_=std::this_thread::get_id();generation_.fetch_add(1,std::memory_order_release);
        active_.store(true,std::memory_order_release);error.clear();return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
void InteractionTrace::Reject(std::uint64_t generation) noexcept{
    const std::lock_guard lock(mutex_);if(!status_.recording||generation!=generation_.load(std::memory_order_relaxed))return;
    active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;status_.loss=true;status_.stop_reason=InteractionStopReason::InvalidEvent;end_ns_=last_ns_;
}
bool InteractionTrace::Append(unsigned char kind,std::span<const std::byte> payload,std::int64_t now_ns,std::uint64_t generation){
    const std::lock_guard lock(mutex_);if(!status_.recording||generation!=generation_.load(std::memory_order_relaxed))return false;
    // An XR observation sampled just before Start can enter after the worker
    // enables recording; it is outside this capture, not a lost control event.
    if(now_ns>=0&&now_ns<start_ns_&&kind!=kControl)return false;
    if(now_ns<start_ns_){active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;status_.loss=true;status_.stop_reason=InteractionStopReason::InvalidEvent;return false;}
    if(now_ns-start_ns_>=kMaxDurationNs){active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;status_.stop_reason=InteractionStopReason::DurationLimit;end_ns_=start_ns_+kMaxDurationNs;return false;}
    if(kind==kControl&&!seeded_){
        if(payload.empty()||payload.front()!=std::byte{0}){
            active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;status_.loss=true;status_.stop_reason=InteractionStopReason::InvalidEvent;return false;
        }
        seeded_=true;
    }
    if(payload.size()>kMaxBytes-kFooterBytes-20||data_.size()>kMaxBytes-kFooterBytes-20-payload.size()){
        active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;status_.loss=true;status_.stop_reason=InteractionStopReason::ByteLimit;end_ns_=last_ns_;return false;
    }
    Writer prefix;prefix.bytes.reserve(20);prefix.U(payload.size()+16,4);prefix.U(kind,1);prefix.U(0,1);prefix.U(0,2);
    prefix.U(status_.event_count+1,4);prefix.I(now_ns);
    data_.insert(data_.end(),prefix.bytes.begin(),prefix.bytes.end());data_.insert(data_.end(),payload.begin(),payload.end());
    ++status_.event_count;if(kind==kControl)++status_.control_count;else if(kind==kInput)++status_.input_count;else ++status_.observation_count;
    last_ns_=std::max(last_ns_,now_ns);end_ns_=last_ns_;
    if(status_.event_count==kMaxEvents){active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;status_.stop_reason=InteractionStopReason::EventLimit;}
    return true;
}
bool InteractionTrace::AppendControl(const FullControlEvent& event,std::int64_t now_ns) noexcept{
    const auto generation=generation_.load(std::memory_order_acquire);
    if(!Recording())return false;
    try{Writer w;ControlArguments(w,event);Output(w,event.expected);return Append(kControl,w.bytes,now_ns,generation);}catch(...){Reject(generation);return false;}
}
bool InteractionTrace::AppendInput(const InteractionInputObservation& event,std::int64_t now_ns) noexcept{
    const auto generation=generation_.load(std::memory_order_acquire);
    if(!Recording())return false;
    try{Writer w;w.Pose(event.head);w.B(event.head_valid);for(auto value:event.raw)Value(w,value);Frame(w,event.input);
        Settings(w,event.settings);w.U(event.settings_generation,8);w.U(event.input_sequence,8);return Append(kInput,w.bytes,now_ns,generation);
    }catch(...){Reject(generation);return false;}
}
bool InteractionTrace::AppendObservation(const InteractionObservation& event,std::int64_t now_ns) noexcept{
    const auto generation=generation_.load(std::memory_order_acquire);
    if(!Recording())return false;
    try{Writer w;const auto kind=static_cast<unsigned char>(event.kind);if(kind>6)throw std::runtime_error("invalid observation kind");
        w.U(kind,1);for(auto value:event.counters)w.U(value,8);for(double value:event.values){if(!std::isfinite(value))throw std::runtime_error("nonfinite observation");w.D(value);}w.U(event.flags,8);
        return Append(kObservation,w.bytes,now_ns,generation);
    }catch(...){Reject(generation);return false;}
}
void InteractionTrace::Stop(std::int64_t now_ns){
    const std::lock_guard lock(mutex_);if(!status_.recording)return;
    active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;
    if(now_ns<start_ns_){status_.loss=true;status_.stop_reason=InteractionStopReason::InvalidEvent;end_ns_=last_ns_;}
    else{end_ns_=std::min(std::max(last_ns_,now_ns),start_ns_+kMaxDurationNs);
        status_.stop_reason=now_ns-start_ns_>=kMaxDurationNs?InteractionStopReason::DurationLimit:InteractionStopReason::Operator;}
}
void InteractionTrace::Expire(std::int64_t now_ns){
    if(!Recording())return;
    const std::lock_guard lock(mutex_);
    if(!status_.recording || now_ns<start_ns_ || now_ns-start_ns_<kMaxDurationNs)return;
    active_.store(false,std::memory_order_release);status_.recording=false;status_.pending_flush=true;
    status_.stop_reason=InteractionStopReason::DurationLimit;end_ns_=start_ns_+kMaxDurationNs;
}
InteractionTraceStatus InteractionTrace::Status()const{const std::lock_guard lock(mutex_);return status_;}
bool InteractionTrace::NeedsFlush()const{const std::lock_guard lock(mutex_);return status_.pending_flush&&!flushing_;}
bool InteractionTrace::Flush(const std::filesystem::path& file,std::string& error){
    Bytes bytes;InteractionTraceStatus status;std::int64_t end=0;
    std::size_t payload_size=0;bool payload_transferred=false;
    try{
        {const std::lock_guard lock(mutex_);if(std::this_thread::get_id()!=worker_){error="Only the recording worker may save";return false;}
         if(status_.recording||!status_.pending_flush||flushing_){error="Recording is not ready to save";return false;}
         // Copy the potentially allocating identity/status before changing
         // ownership. A preparation failure must leave a retryable recording.
         status=status_;end=end_ns_;payload_size=data_.size();
         bytes=std::move(data_);flushing_=true;payload_transferred=true;}
        Writer footer;footer.Magic("QEND");footer.U(status.event_count,4);footer.U(status.control_count,4);footer.U(status.input_count,4);footer.U(status.observation_count,4);
        footer.U(static_cast<unsigned char>(status.stop_reason),1);footer.B(status.loss);footer.U(0,2);footer.I(end);
        bytes.insert(bytes.end(),footer.bytes.begin(),footer.bytes.end());Writer sum;sum.U(Checksum(bytes),8);bytes.insert(bytes.end(),sum.bytes.begin(),sum.bytes.end());
        auto temporary=file;temporary+=".pending";
        {std::ofstream out(temporary,std::ios::binary|std::ios::trunc);out.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));out.close();
         if(!out)throw std::runtime_error("Interaction recording write failed");}
        std::filesystem::rename(temporary,file);
        {const std::lock_guard lock(mutex_);status_.saved=true;status_.pending_flush=false;flushing_=false;}
        error.clear();return true;
    }catch(const std::exception& e){
        if(payload_transferred){
            bytes.resize(payload_size);const std::lock_guard lock(mutex_);data_=std::move(bytes);flushing_=false;
        }
        error=e.what();return false;
    }
}

InteractionReplayResult ReplayInteractionTrace(std::span<const std::byte> data){
    InteractionReplayResult result;
    try{
        if(data.size()>InteractionTrace::kMaxBytes||data.size()<12+kFooterBytes)throw std::runtime_error("invalid trace file size");
        Reader checksum{data.subspan(data.size()-8)};
        if(Checksum(data.first(data.size()-8))!=checksum.U(8))throw std::runtime_error("trace checksum mismatch");
        Reader footer{data.last(kFooterBytes)};footer.Magic("QEND");
        const auto events=footer.U(4),controls=footer.U(4),inputs=footer.U(4),observations=footer.U(4);
        const auto reason=footer.U(1);const bool loss=footer.B();if(footer.U(2)||reason>5)throw std::runtime_error("invalid trace footer flags");
        const auto end=footer.I();footer.U(8);
        if(loss||reason>=static_cast<unsigned char>(InteractionStopReason::ByteLimit))throw std::runtime_error("recording lost events or contains invalid append");
        if(events>InteractionTrace::kMaxEvents||events!=controls+inputs+observations)throw std::runtime_error("invalid trace event counts");
        Reader r{data.first(data.size()-kFooterBytes)};r.Magic("QITR");
        const auto version=r.U(2);
        if((version<1 || version>3)||r.U(2))throw std::runtime_error("unsupported trace version or flags");
        const auto header_size=r.U(4);const auto start=r.I();result.run_id=r.Text(64);
        if(!RunId(result.run_id)||start<0||start>std::numeric_limits<std::int64_t>::max()-InteractionTrace::kMaxDurationNs||
           end<start||end-start>InteractionTrace::kMaxDurationNs)throw std::runtime_error("invalid recording identity/duration");
        if((reason==static_cast<unsigned char>(InteractionStopReason::EventLimit)&&events!=InteractionTrace::kMaxEvents)||
           (reason==static_cast<unsigned char>(InteractionStopReason::DurationLimit)&&end-start!=InteractionTrace::kMaxDurationNs))
            throw std::runtime_error("invalid capacity stop marker");
        Settings(r,version);if(r.at!=header_size)throw std::runtime_error("invalid trace header size");
        FullControlMachine machine;std::int64_t last_control_time=start;
        while(!r.Done()){
            if(result.event_count>=events)throw std::runtime_error("more records than the bounded footer count");
            const auto length=static_cast<std::size_t>(r.U(4));
            if(length<16||length>r.bytes.size()-r.at)throw std::runtime_error("truncated trace record");
            Reader record{r.bytes.subspan(r.at,length)};r.at+=length;
            const auto kind=record.U(1);if(record.U(1)||record.U(2))throw std::runtime_error("unknown record flags");
            if(record.U(4)!=static_cast<std::uint64_t>(result.event_count)+1)throw std::runtime_error("missing or reordered event sequence");
            const auto now=record.I();if(now<start||now>end)throw std::runtime_error("event outside recording time range");
            ++result.event_count;
            if(kind==kControl){
                const auto event=ControlArguments(record);
                if(!result.control_count&&event.operation!=FullControlOperation::Reset)throw std::runtime_error("trace lacks explicit Reset seed");
                if(now<last_control_time)throw std::runtime_error("control operation time moved backwards");last_control_time=now;
                Apply(machine,event);Writer actual;Output(actual,machine.Output());
                const auto expected=record.bytes.subspan(record.at);
                if(expected.size()!=actual.bytes.size()||!std::equal(expected.begin(),expected.end(),actual.bytes.begin()))
                    throw std::runtime_error("control output mismatch at event "+std::to_string(result.event_count));
                record.at=record.bytes.size();++result.control_count;
            }else if(kind==kInput){
                record.Pose();record.B();for(std::size_t i=0;i<kInputCount;++i)Value(record);Frame(record);Settings(record,version);record.U(8);record.U(8);++result.input_count;
            }else if(kind==kObservation){
                if(record.U(1)>6)throw std::runtime_error("unknown external observation");
                for(unsigned i=0;i<4;++i)record.U(8);
                for(unsigned i=0;i<4;++i)if(!std::isfinite(record.D()))throw std::runtime_error("nonfinite external observation");record.U(8);++result.external_observations;
            }else throw std::runtime_error("unknown trace event kind");
            if(!record.Done())throw std::runtime_error("trailing event bytes");
        }
        if(result.event_count!=events||result.control_count!=controls||result.input_count!=inputs||result.external_observations!=observations||!controls)
            throw std::runtime_error("trace footer count mismatch or missing Reset seed");
        result.passed=true;
    }catch(const std::exception& e){result.error=e.what();}
    return result;
}
InteractionReplayResult ReplayInteractionFile(const std::filesystem::path& file){
    try{
        const auto size=std::filesystem::file_size(file);if(size>InteractionTrace::kMaxBytes)throw std::runtime_error("trace exceeds 32 MiB");
        Bytes bytes(static_cast<std::size_t>(size));std::ifstream in(file,std::ios::binary);
        in.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
        if(!in||in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trace read failed or file changed");
        return ReplayInteractionTrace(bytes);
    }catch(const std::exception& e){InteractionReplayResult result;result.error=e.what();return result;}
}
} // namespace quest_newton
