#include "remote_scene_receiver.h"
#include "remote_scene.h"
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <vector>

namespace quest_newton {
namespace {
void Check(JNIEnv* env,const char* operation){
    if(!env->ExceptionCheck())return;
    auto exception=env->ExceptionOccurred();env->ExceptionClear();
    std::string message=operation;
    auto type=env->GetObjectClass(exception);
    auto method=type?env->GetMethodID(type,"getMessage","()Ljava/lang/String;"):nullptr;
    auto text=method?static_cast<jstring>(env->CallObjectMethod(exception,method)):nullptr;
    if(text&&!env->ExceptionCheck()){
        const char* chars=env->GetStringUTFChars(text,nullptr);
        if(chars){
            try{message+="; ";message.append(chars,std::min<std::size_t>(std::char_traits<char>::length(chars),256));}
            catch(...){env->ReleaseStringUTFChars(text,chars);throw;}
            env->ReleaseStringUTFChars(text,chars);
        }
    }
    if(env->ExceptionCheck())env->ExceptionClear();
    throw std::runtime_error(message);
}
struct LocalFrame {
    JNIEnv* env;
    explicit LocalFrame(JNIEnv* value):env(value){if(env->PushLocalFrame(32)!=JNI_OK){Check(env,"Remote scene JNI frame failed");throw std::runtime_error("Remote scene JNI frame failed");}}
    ~LocalFrame(){env->PopLocalFrame(nullptr);}
};
jmethodID Method(JNIEnv* env,jclass type,const char* name,const char* signature){
    auto result=env->GetMethodID(type,name,signature);Check(env,"Remote scene Java method unavailable");
    if(!result)throw std::runtime_error("Remote scene Java method unavailable");return result;
}
jclass LoadClass(JNIEnv* env,jobject activity){
    auto activity_type=env->GetObjectClass(activity);Check(env,"Remote scene Activity unavailable");
    auto loader=env->CallObjectMethod(activity,Method(env,activity_type,"getClassLoader","()Ljava/lang/ClassLoader;"));Check(env,"Remote scene class loader unavailable");
    if(!loader)throw std::runtime_error("Remote scene class loader unavailable");
    auto loader_type=env->GetObjectClass(loader);Check(env,"Remote scene class loader unavailable");
    auto name=env->NewStringUTF("com.questnewton.RemoteSceneConnection");Check(env,"Remote scene class name allocation failed");
    auto local=env->CallObjectMethod(loader,Method(env,loader_type,"loadClass","(Ljava/lang/String;)Ljava/lang/Class;"),name);
    Check(env,"Remote scene Java class unavailable");
    auto result=static_cast<jclass>(env->NewGlobalRef(local));Check(env,"Remote scene class reference failed");
    if(!result)throw std::runtime_error("Remote scene class reference failed");return result;
}
bool Attach(JavaVM* vm,JNIEnv*& env,bool& attached){
    const auto result=vm->GetEnv(reinterpret_cast<void**>(&env),JNI_VERSION_1_6);
    if(result==JNI_OK)return true;
    if(result!=JNI_EDETACHED)return false;
    if(vm->AttachCurrentThread(&env,nullptr)!=JNI_OK)return false;
    attached=true;return true;
}
} // namespace

RemoteSceneReceiver::~RemoteSceneReceiver(){
    auto* vm=vm_;JNIEnv* env=nullptr;bool attached=false;
    if(vm)Attach(vm,env,attached);
    Shutdown(env);
    if(attached)vm->DetachCurrentThread();
}
bool RemoteSceneReceiver::Init(JavaVM* vm,JNIEnv* env,jobject activity,std::string& error){
    if(connection_class_){error.clear();return true;}
    if(!vm||!env||!activity){error="Remote scene Java context unavailable";return false;}
    try {
        LocalFrame frame(env);vm_=vm;
        context_=env->NewGlobalRef(activity);Check(env,"Remote scene context reference failed");
        if(!context_)throw std::runtime_error("Remote scene context reference failed");
        connection_class_=LoadClass(env,activity);
        constructor_=Method(env,connection_class_,"<init>","(Landroid/content/Context;I)V");
        connect_=Method(env,connection_class_,"connect","()V");
        read_=Method(env,connection_class_,"readFrame","()[B");
        close_=Method(env,connection_class_,"close","()V");
        error.clear();SetStatus("Remote disconnected");return true;
    }catch(const std::exception& failure){Shutdown(env);error=failure.what();SetStatus(error);return false;}
}
bool RemoteSceneReceiver::Start(JNIEnv* env,std::string& error){return StartImpl(env,0,error);}
bool RemoteSceneReceiver::StartDemo(JNIEnv* env,std::string& error,RemoteSceneRepresentation representation,bool rgb){
    const auto value=static_cast<unsigned>(representation);
    if(value>2 || (rgb && value==0)){error="Unknown demo representation";return false;}
    return StartImpl(env,static_cast<int>(value)+(rgb?3:1),error);
}
bool RemoteSceneReceiver::StartImpl(JNIEnv* env,int demo_mode,std::string& error){
    const bool demo=demo_mode!=0;
    if(Running()){error="Remote receiver is already running; disconnect first";return false;}
    if(!env||!vm_||!context_||!connection_class_){error="Remote scene receiver is not initialized";return false;}
    Stop(env); // Reap a completed previous connection; never implicitly replace a running one.
    try {
        LocalFrame frame(env);
        auto object=env->NewObject(connection_class_,constructor_,context_,static_cast<jint>(demo_mode));
        Check(env,"Remote scene connection allocation failed");
        connection_=env->NewGlobalRef(object);Check(env,"Remote scene connection reference failed");
        if(!connection_)throw std::runtime_error("Remote scene connection reference failed");
        // The owner can now close this Java object even before connect begins.
        const auto generation=mailbox_.Reset();stop_=false;running_=true;
        SetStatus(demo?"Loading generated remote demo":"Connecting to configured remote source");
        worker_=std::thread(&RemoteSceneReceiver::Run,this,connection_,generation,demo);
        error.clear();return true;
    }catch(const std::exception& failure){Stop(env);error=failure.what();SetStatus(error);return false;}
}
void RemoteSceneReceiver::Stop(JNIEnv* env){
    stop_=true;
    if(env&&connection_&&close_){env->CallVoidMethod(connection_,close_);if(env->ExceptionCheck())env->ExceptionClear();}
    if(worker_.joinable())worker_.join();
    if(env&&connection_)env->DeleteGlobalRef(connection_);
    connection_=nullptr;running_=false;SetStatus("Remote disconnected");
}
void RemoteSceneReceiver::Shutdown(JNIEnv* env){
    Stop(env);
    if(env&&connection_class_)env->DeleteGlobalRef(connection_class_);
    if(env&&context_)env->DeleteGlobalRef(context_);
    connection_class_=nullptr;context_=nullptr;vm_=nullptr;
    constructor_=connect_=read_=close_=nullptr;
}
std::string RemoteSceneReceiver::Status()const{const std::lock_guard lock(status_mutex_);return status_;}
void RemoteSceneReceiver::SetStatus(std::string_view text)noexcept{
    try{const std::lock_guard lock(status_mutex_);status_.assign(text.data(),text.size());}catch(...){}
}
void RemoteSceneReceiver::Run(jobject connection,std::uint64_t generation,bool demo)noexcept{
    JNIEnv* env=nullptr;bool attached=false,loaded=false;
    try {
        if(!Attach(vm_,env,attached))throw std::runtime_error("Remote scene worker cannot attach to Java");
        {
            LocalFrame frame(env);
            if(!stop_.load()){env->CallVoidMethod(connection,connect_);Check(env,"Remote scene open failed");}
        }
        while(!stop_.load()){
            LocalFrame frame(env);
            auto bytes=static_cast<jbyteArray>(env->CallObjectMethod(connection,read_));Check(env,"Remote scene read failed");
            if(!bytes){SetStatus(demo&&loaded?"Generated demo loaded (static)":"Remote source ended");break;}
            const auto size=env->GetArrayLength(bytes);Check(env,"Remote scene byte array invalid");
            if(size<static_cast<jsize>(kRemoteSceneHeaderBytes)||size>static_cast<jsize>(kRemoteSceneMaxPacketBytes))
                throw std::runtime_error("Remote scene packet exceeds decoded bounds");
            std::vector<std::byte> packet(static_cast<std::size_t>(size));
            env->GetByteArrayRegion(bytes,0,size,reinterpret_cast<jbyte*>(packet.data()));Check(env,"Remote scene packet copy failed");
            if(stop_.load())break;
            std::string error;
            if(!mailbox_.Publish(packet,std::chrono::steady_clock::now(),generation,error))throw std::runtime_error("Remote scene rejected: "+error);
            loaded=true;SetStatus(demo?"Generated demo loaded (static)":"Remote receiving");
        }
    }catch(const std::exception& failure){if(!stop_.load())try{SetStatus(failure.what());}catch(...){}
    }catch(...){if(!stop_.load())try{SetStatus("Remote scene receiver failed");}catch(...){}}
    if(env&&close_){env->CallVoidMethod(connection,close_);if(env->ExceptionCheck())env->ExceptionClear();}
    if(attached)vm_->DetachCurrentThread();
    running_=false;
}
} // namespace quest_newton
