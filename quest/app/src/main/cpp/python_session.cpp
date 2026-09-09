#include "python_session.h"
#include <array>
#include <stdexcept>

namespace quest_newton {
namespace {
void Check(JNIEnv* env) {
    if(!env->ExceptionCheck())return;
    auto exception=env->ExceptionOccurred();env->ExceptionClear();
    std::string message="Python/JNI exception";
    auto cls=env->GetObjectClass(exception);
    auto method=cls?env->GetMethodID(cls,"toString","()Ljava/lang/String;"):nullptr;
    auto text=method?static_cast<jstring>(env->CallObjectMethod(exception,method)):nullptr;
    if(text && !env->ExceptionCheck()) {
        const char* chars=env->GetStringUTFChars(text,nullptr);
        if(chars){message=chars;env->ReleaseStringUTFChars(text,chars);}
    }
    if(env->ExceptionCheck())env->ExceptionClear();
    throw std::runtime_error(message);
}
struct Frame {
    JNIEnv* env;
    explicit Frame(JNIEnv* e):env(e){if(env->PushLocalFrame(64)!=JNI_OK){Check(env);throw std::runtime_error("JNI local frame failed");}}
    ~Frame(){env->PopLocalFrame(nullptr);}
};
jmethodID Method(JNIEnv* env,jclass cls,const char* name,const char* signature) {
    const auto id=env->GetMethodID(cls,name,signature);Check(env);
    if(!id)throw std::runtime_error(std::string("Missing bridge method ")+name);return id;
}
void CreateFloatBuffer(JNIEnv* env,jfloatArray& buffer,jsize size) {
    auto array=env->NewFloatArray(size);Check(env);
    if(!array)throw std::runtime_error("JNI array allocation failed");
    buffer=static_cast<jfloatArray>(env->NewGlobalRef(array));Check(env);
    if(!buffer)throw std::runtime_error("Cannot retain JNI input array");
}
void CopyFloats(JNIEnv* env,jfloatArray array,std::span<const float> values) {
    env->SetFloatArrayRegion(array,0,static_cast<jsize>(values.size()),values.data());Check(env);
}
}
jclass LoadSimulationBridge(JNIEnv* env,jobject activity) {
    Frame frame(env);
    auto activity_class=env->GetObjectClass(activity);Check(env);
    auto loader=env->CallObjectMethod(activity,Method(env,activity_class,"getClassLoader","()Ljava/lang/ClassLoader;"));Check(env);
    auto loader_class=env->GetObjectClass(loader);Check(env);
    auto name=env->NewStringUTF("com.questnewton.SimulationBridge");Check(env);
    auto cls=env->CallObjectMethod(loader,Method(env,loader_class,"loadClass","(Ljava/lang/String;)Ljava/lang/Class;"),name);Check(env);
    auto result=static_cast<jclass>(env->NewGlobalRef(cls));Check(env);
    if(!result)throw std::runtime_error("Cannot retain bridge class");return result;
}
PythonSession::PythonSession(JavaVM* vm,jobject activity,jclass bridge,const std::string& settings):vm_(vm) {
    const auto status=vm_->GetEnv(reinterpret_cast<void**>(&env_),JNI_VERSION_1_6);
    if(status==JNI_EDETACHED){if(vm_->AttachCurrentThread(&env_,nullptr)!=JNI_OK)throw std::runtime_error("Cannot attach simulation worker");attached_=true;}
    else if(status!=JNI_OK)throw std::runtime_error("Cannot get worker JNI environment");
    try {
        Frame frame(env_);
        CreateFloatBuffer(env_,targets_input_,static_cast<jsize>(kinematics::kHome.size()));
        CreateFloatBuffer(env_,base_input_,7);
        const auto constructor=Method(env_,bridge,"<init>","(Landroid/content/Context;Ljava/lang/String;)V");
        auto config=env_->NewStringUTF(settings.c_str());Check(env_);
        auto local=env_->NewObject(bridge,constructor,activity,config);Check(env_);
        object_=env_->NewGlobalRef(local);Check(env_);if(!object_)throw std::runtime_error("Cannot retain simulation bridge");
        snapshot_=Method(env_,bridge,"snapshot","()[B");
        step_=Method(env_,bridge,"step","(D[FFZ)[B");
        command_=Method(env_,bridge,"command","(Ljava/lang/String;)[B");
        base_=Method(env_,bridge,"setBasePose","([F)[B");
        environment_=Method(env_,bridge,"setEnvironment","(Ljava/lang/String;)[B");
        details_=Method(env_,bridge,"details","(Z)[B");
        configure_=Method(env_,bridge,"configure","(Ljava/lang/String;)V");
        metadata_=Method(env_,bridge,"metadata","()Ljava/lang/String;");
        gripper_status_=Method(env_,bridge,"gripperStatus","()Ljava/lang/String;");
        close_=Method(env_,bridge,"close","()V");
    }catch(...){
        if(object_)env_->DeleteGlobalRef(object_);
        if(targets_input_)env_->DeleteGlobalRef(targets_input_);
        if(base_input_)env_->DeleteGlobalRef(base_input_);
        if(attached_)vm_->DetachCurrentThread();
        throw;
    }
}
PythonSession::~PythonSession(){
    if(object_){if(close_)env_->CallVoidMethod(object_,close_);if(env_->ExceptionCheck())env_->ExceptionClear();env_->DeleteGlobalRef(object_);}
    if(targets_input_)env_->DeleteGlobalRef(targets_input_);
    if(base_input_)env_->DeleteGlobalRef(base_input_);
    if(attached_)vm_->DetachCurrentThread();
}
std::vector<std::byte> PythonSession::Bytes(jobject object) {
    Check(env_);if(!object)throw std::runtime_error("Missing scene snapshot");
    const auto array=static_cast<jbyteArray>(object);const auto size=env_->GetArrayLength(array);Check(env_);
    if(size<48 || size>65536)throw std::runtime_error("Scene snapshot size outside contract");
    std::vector<std::byte> bytes(static_cast<std::size_t>(size));
    env_->GetByteArrayRegion(array,0,size,reinterpret_cast<jbyte*>(bytes.data()));Check(env_);return bytes;
}
std::vector<std::byte> PythonSession::Snapshot(){Frame frame(env_);return Bytes(env_->CallObjectMethod(object_,snapshot_));}
std::vector<std::byte> PythonSession::Details(bool contacts){Frame frame(env_);return Bytes(env_->CallObjectMethod(object_,details_,static_cast<jboolean>(contacts)));}
std::vector<std::byte> PythonSession::Step(double dt,const kinematics::JointVector& targets,float gripper,bool gripper_input_allowed){
    Frame frame(env_);CopyFloats(env_,targets_input_,targets);
    return Bytes(env_->CallObjectMethod(object_,step_,dt,targets_input_,static_cast<jfloat>(gripper),static_cast<jboolean>(gripper_input_allowed)));
}
std::vector<std::byte> PythonSession::Command(const std::string& command){
    Frame frame(env_);auto text=env_->NewStringUTF(command.c_str());Check(env_);return Bytes(env_->CallObjectMethod(object_,command_,text));
}
std::vector<std::byte> PythonSession::SetBasePose(const kinematics::Pose& pose){
    Frame frame(env_);const std::array<float,7> values{pose.position[0],pose.position[1],pose.position[2],pose.rotation[0],pose.rotation[1],pose.rotation[2],pose.rotation[3]};
    CopyFloats(env_,base_input_,values);
    return Bytes(env_->CallObjectMethod(object_,base_,base_input_));
}
void PythonSession::Configure(const std::string& settings){
    Frame frame(env_);auto text=env_->NewStringUTF(settings.c_str());Check(env_);env_->CallVoidMethod(object_,configure_,text);Check(env_);
}
std::vector<std::byte> PythonSession::SetEnvironment(const std::string& room){
    Frame frame(env_);auto text=env_->NewStringUTF(room.c_str());Check(env_);
    return Bytes(env_->CallObjectMethod(object_,environment_,text));
}
std::string PythonSession::Metadata(){
    Frame frame(env_);auto text=static_cast<jstring>(env_->CallObjectMethod(object_,metadata_));Check(env_);
    if(!text)throw std::runtime_error("Missing runtime metadata");const char* chars=env_->GetStringUTFChars(text,nullptr);Check(env_);
    if(!chars)throw std::runtime_error("Cannot read runtime metadata");std::string result(chars);env_->ReleaseStringUTFChars(text,chars);return result;
}
std::string PythonSession::GripperStatus(){
    Frame frame(env_);auto text=static_cast<jstring>(env_->CallObjectMethod(object_,gripper_status_));Check(env_);
    if(!text)throw std::runtime_error("Missing gripper status");const char* chars=env_->GetStringUTFChars(text,nullptr);Check(env_);
    if(!chars)throw std::runtime_error("Cannot read gripper status");std::string result(chars);env_->ReleaseStringUTFChars(text,chars);return result;
}
} // namespace quest_newton
