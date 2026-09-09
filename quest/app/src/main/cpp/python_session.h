#pragma once
#include "quest_newton/franka_kinematics.h"
#include <jni.h>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace quest_newton {
// Class must be resolved through the Activity's class loader before worker use.
jclass LoadSimulationBridge(JNIEnv* env,jobject activity);
class PythonSession {
public:
    PythonSession(JavaVM* vm,jobject activity,jclass bridge,const std::string& settings);
    ~PythonSession();
    PythonSession(const PythonSession&)=delete;
    PythonSession& operator=(const PythonSession&)=delete;
    std::vector<std::byte> Snapshot();
    std::vector<std::byte> Step(double dt,const kinematics::JointVector& targets,float gripper,bool gripper_input_allowed);
    std::vector<std::byte> Command(const std::string& command);
    std::vector<std::byte> SetBasePose(const kinematics::Pose& pose);
    std::vector<std::byte> SetEnvironment(const std::string& room);
    std::vector<std::byte> Details(bool contacts);
    void Configure(const std::string& settings);
    std::string Metadata();
    std::string GripperStatus();
private:
    std::vector<std::byte> Bytes(jobject array);
    JavaVM* vm_=nullptr;
    JNIEnv* env_=nullptr;
    jobject object_=nullptr;
    // Worker-owned inputs: Java/Python consume and copy them synchronously.
    jfloatArray targets_input_=nullptr,base_input_=nullptr;
    bool attached_=false;
    jmethodID snapshot_=nullptr,step_=nullptr,command_=nullptr,base_=nullptr,environment_=nullptr,configure_=nullptr,metadata_=nullptr,close_=nullptr;
    jmethodID details_=nullptr,gripper_status_=nullptr;
};
} // namespace quest_newton
