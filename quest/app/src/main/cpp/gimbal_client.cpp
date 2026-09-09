#include "gimbal_client.h"
#include <cmath>
#include <limits>
#include <stdexcept>
namespace quest_newton {
bool GimbalClientPolicy::Accept(std::span<const std::byte> bytes,GimbalTime received,std::string& error){
    GimbalFeedback next;if(!DecodeGimbalFeedback(bytes,next,error))return false;
    const auto reject=[&](const char* text){error=text;return false;};
    const bool armed=(next.flags&2U)!=0,lease=(next.flags&8U)!=0;
    if(next.pan<next.pan_min||next.pan>next.pan_max||next.tilt<next.tilt_min||next.tilt>next.tilt_max||
       armed!=lease||(armed&&!(next.flags&4U))||(!lease&&next.lease_ms))return reject("Inconsistent simulated camera position or lease feedback");
    if(!initialized_){if(next.ack||armed)return reject("Initial camera session must be unarmed with zero acknowledgement");}
    else{
        if(!pending_||next.ack!=sequence_||next.session!=feedback_.session||next.source_id!=feedback_.source_id||next.challenge==feedback_.challenge||
           next.server_ns<feedback_.server_ns||received<feedback_.received||next.map_generation<feedback_.map_generation)
            return reject("Camera session, challenge or acknowledgement continuity failed");
        if(next.pan_min!=feedback_.pan_min||next.pan_max!=feedback_.pan_max||next.tilt_min!=feedback_.tilt_min||next.tilt_max!=feedback_.tilt_max||next.max_rate!=feedback_.max_rate)
            return reject("Camera limits changed within a control session");
        if(pending_operation_!=GimbalOperation::Aim&&armed)return reject("Camera remained armed after Hold or reset");
        if(pending_operation_==GimbalOperation::Aim&&!armed)needs_release_=true;
        if(pending_release_)needs_release_=false;
    }
    next.received=received;feedback_=next;initialized_=true;pending_=false;error.clear();return true;
}
std::array<std::byte,64> GimbalClientPolicy::Command(const GimbalIntent& latest,bool reset_map,GimbalTime now,std::uint64_t stamp){
    if(!initialized_||pending_||sequence_==std::numeric_limits<std::uint64_t>::max()||!stamp||stamp<=client_ns_)
        throw std::logic_error("Camera exchange sequence or client clock is invalid");
    const bool feedback_fresh=now>=feedback_.received&&now-feedback_.received<=std::chrono::milliseconds(100);
    const bool fresh=latest.sampled!=GimbalTime{}&&now>=latest.sampled&&now-latest.sampled<=std::chrono::milliseconds(100);
    const bool release=fresh&&latest.operation==GimbalOperation::Hold&&!latest.clutch;
    const bool aim=fresh&&latest.operation==GimbalOperation::Aim&&latest.clutch&&std::isfinite(latest.pan)&&std::isfinite(latest.tilt)&&
        latest.pan>=feedback_.pan_min&&latest.pan<=feedback_.pan_max&&latest.tilt>=feedback_.tilt_min&&latest.tilt<=feedback_.tilt_max;
    GimbalIntent next;next.sampled=now;next.pan=feedback_.pan;next.tilt=feedback_.tilt;
    pending_release_=false;
    if(reset_map&&feedback_fresh){next.operation=GimbalOperation::ResetMap;needs_release_=true;}
    else if(aim&&feedback_fresh&&!needs_release_&&(feedback_.flags&4U)){next=latest;}
    else{if(!release)needs_release_=true;pending_release_=release&&feedback_fresh;}
    pending_operation_=next.operation;client_ns_=stamp;pending_=true;
    return EncodeGimbalCommand(feedback_,next,++sequence_,stamp);
}
}

#if defined(__ANDROID__)
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
namespace quest_newton {
namespace {
void Check(JNIEnv* env,const char* operation){if(env->ExceptionCheck()){env->ExceptionClear();throw std::runtime_error(operation);}}
struct LocalFrame {
    JNIEnv* env;
    explicit LocalFrame(JNIEnv* value):env(value){if(env->PushLocalFrame(32)!=JNI_OK){Check(env,"Camera control JNI frame failed");throw std::runtime_error("Camera control JNI frame failed");}}
    ~LocalFrame(){env->PopLocalFrame(nullptr);}
};
jmethodID Method(JNIEnv* env,jclass type,const char* name,const char* signature){
    auto result=env->GetMethodID(type,name,signature);Check(env,"Camera control Java method unavailable");
    if(!result)throw std::runtime_error("Camera control Java method unavailable");return result;
}
jclass LoadClass(JNIEnv* env,jobject activity){
    auto activity_type=env->GetObjectClass(activity);Check(env,"Camera control Activity unavailable");
    auto loader=env->CallObjectMethod(activity,Method(env,activity_type,"getClassLoader","()Ljava/lang/ClassLoader;"));Check(env,"Camera control class loader unavailable");
    if(!loader)throw std::runtime_error("Camera control class loader unavailable");
    auto loader_type=env->GetObjectClass(loader);Check(env,"Camera control class loader unavailable");
    auto name=env->NewStringUTF("com.questnewton.GimbalControlConnection");Check(env,"Camera control class name allocation failed");
    auto local=env->CallObjectMethod(loader,Method(env,loader_type,"loadClass","(Ljava/lang/String;)Ljava/lang/Class;"),name);
    Check(env,"Camera control Java class unavailable");
    auto result=static_cast<jclass>(env->NewGlobalRef(local));Check(env,"Camera control class reference failed");
    if(!result)throw std::runtime_error("Camera control class reference failed");return result;
}
bool Attach(JavaVM* vm,JNIEnv*& env,bool& attached){
    const auto result=vm->GetEnv(reinterpret_cast<void**>(&env),JNI_VERSION_1_6);
    if(result==JNI_OK)return true;
    if(result!=JNI_EDETACHED)return false;
    if(vm->AttachCurrentThread(&env,nullptr)!=JNI_OK)return false;
    attached=true;return true;
}
} // namespace

struct GimbalControlClient::Impl {
    JavaVM* vm=nullptr; jobject context=nullptr,connection=nullptr;jclass type=nullptr;
    jmethodID constructor=nullptr,open=nullptr,exchange=nullptr,close=nullptr;
    std::thread worker;std::atomic<bool> stop{true};mutable std::mutex mutex;std::condition_variable wake;
    GimbalClientSnapshot state;GimbalIntent latest;GimbalTime reset_requested{};
    void Run() noexcept {
        JNIEnv* env=nullptr;bool attached=false;
        const auto read=[&](jbyteArray array){
            Check(env,"Camera feedback read failed");
            if(!array||env->GetArrayLength(array)!=96)throw std::runtime_error("Camera feedback must contain 96 bytes");
            std::array<std::byte,96> data{};env->GetByteArrayRegion(array,0,96,reinterpret_cast<jbyte*>(data.data()));Check(env,"Camera feedback copy failed");return data;
        };
        try{
            if(!Attach(vm,env,attached))throw std::runtime_error("Camera control worker cannot attach to Java");
            GimbalClientPolicy policy;std::string error;
            {
                LocalFrame frame(env);if(stop.load())throw std::runtime_error("Camera connection cancelled");
                const auto data=read(static_cast<jbyteArray>(env->CallObjectMethod(connection,open)));
                if(!policy.Accept(data,std::chrono::steady_clock::now(),error))throw std::runtime_error(error);
                const std::lock_guard lock(mutex);if(stop.load())throw std::runtime_error("Camera connection cancelled");
                state.connected=true;state.feedback=policy.Feedback();state.error.clear();
            }
            auto next_send=std::chrono::steady_clock::now();
            while(!stop.load()){
                GimbalIntent intent;bool reset=false;
                {
                    std::unique_lock lock(mutex);wake.wait_until(lock,next_send,[&]{return stop.load();});if(stop.load())break;
                    const auto now=std::chrono::steady_clock::now();intent=latest;
                    reset=reset_requested!=GimbalTime{}&&now>=reset_requested&&now-reset_requested<=std::chrono::milliseconds(100);reset_requested={};
                }
                LocalFrame frame(env);const auto sent=std::chrono::steady_clock::now();next_send=sent+std::chrono::milliseconds(50);
                const auto ticks=std::chrono::duration_cast<std::chrono::nanoseconds>(sent.time_since_epoch()).count();
                if(ticks<=0)throw std::runtime_error("Camera control monotonic clock unavailable");
                const auto command=policy.Command(intent,reset,sent,static_cast<std::uint64_t>(ticks));
                auto bytes=env->NewByteArray(64);Check(env,"Camera command array allocation failed");
                if(!bytes)throw std::runtime_error("Camera command array allocation failed");
                env->SetByteArrayRegion(bytes,0,64,reinterpret_cast<const jbyte*>(command.data()));Check(env,"Camera command copy failed");
                if(stop.load())break;
                const auto data=read(static_cast<jbyteArray>(env->CallObjectMethod(connection,exchange,bytes)));
                if(!policy.Accept(data,std::chrono::steady_clock::now(),error))throw std::runtime_error(error);
                const std::lock_guard lock(mutex);if(stop.load())break;state.feedback=policy.Feedback();
            }
        }catch(const std::exception& error){if(!stop.load()){const std::lock_guard lock(mutex);state.error=error.what();}}
        catch(...){if(!stop.load()){const std::lock_guard lock(mutex);state.error="Camera control worker failed";}}
        if(env&&connection&&close){env->CallVoidMethod(connection,close);if(env->ExceptionCheck())env->ExceptionClear();}
        if(attached)vm->DetachCurrentThread();
        const std::lock_guard lock(mutex);state.running=state.connected=false;latest={};reset_requested={};
    }
};
GimbalControlClient::GimbalControlClient():impl_(std::make_unique<Impl>()){}
GimbalControlClient::~GimbalControlClient(){
    auto* vm=impl_->vm;JNIEnv* env=nullptr;bool attached=false;if(vm)Attach(vm,env,attached);Shutdown(env);if(attached)vm->DetachCurrentThread();
}
bool GimbalControlClient::Init(JavaVM* vm,JNIEnv* env,jobject activity,std::string& error){
    auto& p=*impl_;if(p.type){error.clear();return true;}
    if(!vm||!env||!activity){error="Camera Java context unavailable";return false;}
    try{
        LocalFrame frame(env);p.vm=vm;p.context=env->NewGlobalRef(activity);Check(env,"Camera context reference failed");
        if(!p.context)throw std::runtime_error("Camera context reference failed");p.type=LoadClass(env,activity);
        p.constructor=Method(env,p.type,"<init>","(Landroid/content/Context;)V");p.open=Method(env,p.type,"open","()[B");
        p.exchange=Method(env,p.type,"exchange","([B)[B");p.close=Method(env,p.type,"close","()V");error.clear();return true;
    }catch(const std::exception& failure){Shutdown(env);error=failure.what();return false;}
}
bool GimbalControlClient::Start(JNIEnv* env,std::string& error){
    auto& p=*impl_;if(Snapshot().running){error="Camera control is already running";return false;}
    if(!env||!p.vm||!p.context||!p.type){error="Camera control is not initialized";return false;}
    Stop(env);
    try{
        LocalFrame frame(env);auto object=env->NewObject(p.type,p.constructor,p.context);Check(env,"Camera connection allocation failed");
        p.connection=env->NewGlobalRef(object);Check(env,"Camera connection reference failed");if(!p.connection)throw std::runtime_error("Camera connection reference failed");
        {const std::lock_guard lock(p.mutex);p.state={};p.state.running=true;p.latest={};p.reset_requested={};p.stop=false;}
        p.worker=std::thread(&Impl::Run,&p);error.clear();return true;
    }catch(const std::exception& failure){Stop(env);error=failure.what();return false;}
}
void GimbalControlClient::Stop(JNIEnv* env){
    auto& p=*impl_;p.stop=true;p.wake.notify_all();
    if(env&&p.connection&&p.close){env->CallVoidMethod(p.connection,p.close);if(env->ExceptionCheck())env->ExceptionClear();}
    if(p.worker.joinable())p.worker.join();if(env&&p.connection)env->DeleteGlobalRef(p.connection);p.connection=nullptr;
    const std::lock_guard lock(p.mutex);p.state={};p.latest={};p.reset_requested={};
}
void GimbalControlClient::Shutdown(JNIEnv* env){
    auto& p=*impl_;Stop(env);if(env&&p.type)env->DeleteGlobalRef(p.type);if(env&&p.context)env->DeleteGlobalRef(p.context);
    p.type=nullptr;p.context=nullptr;p.vm=nullptr;p.constructor=p.open=p.exchange=p.close=nullptr;
}
void GimbalControlClient::Submit(const GimbalIntent& intent){const std::lock_guard lock(impl_->mutex);if(impl_->state.running)impl_->latest=intent;}
bool GimbalControlClient::RequestMapReset(){
    auto& p=*impl_;const auto now=std::chrono::steady_clock::now();const std::lock_guard lock(p.mutex);
    if(!p.state.connected||now<p.state.feedback.received||now-p.state.feedback.received>std::chrono::milliseconds(100))return false;
    p.reset_requested=now;p.latest={};return true;
}
GimbalClientSnapshot GimbalControlClient::Snapshot()const{const std::lock_guard lock(impl_->mutex);return impl_->state;}
} // namespace quest_newton
#endif
