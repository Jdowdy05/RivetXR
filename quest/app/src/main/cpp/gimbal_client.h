#pragma once
#include "gimbal_control.h"
#include <memory>

namespace quest_newton {
// Pure session policy shared by the joined transport worker and host checks.
class GimbalClientPolicy {
public:
    bool Accept(std::span<const std::byte> bytes,GimbalTime received,std::string& error);
    std::array<std::byte,64> Command(const GimbalIntent& latest,bool reset_map,GimbalTime now,std::uint64_t client_ns);
    const GimbalFeedback& Feedback() const{return feedback_;}
private:
    GimbalFeedback feedback_;
    bool initialized_=false,pending_=false,needs_release_=true,pending_release_=false;
    GimbalOperation pending_operation_=GimbalOperation::Hold;
    std::uint64_t sequence_=0,client_ns_=0;
};
struct GimbalClientSnapshot {
    bool running=false,connected=false;
    GimbalFeedback feedback;
    std::string error;
};
}

#if defined(__ANDROID__)
#include <jni.h>
namespace quest_newton {
// Lifecycle methods are serialized by XR ownership. All config/TLS/RPC work is
// on one joined worker; Stop closes Java I/O before joining and clears intent.
class GimbalControlClient {
public:
    GimbalControlClient();
    ~GimbalControlClient();
    GimbalControlClient(const GimbalControlClient&)=delete;
    GimbalControlClient& operator=(const GimbalControlClient&)=delete;
    bool Init(JavaVM*,JNIEnv*,jobject activity,std::string& error);
    bool Start(JNIEnv*,std::string& error);
    void Stop(JNIEnv*);
    void Shutdown(JNIEnv*);
    void Submit(const GimbalIntent&);
    bool RequestMapReset();
    GimbalClientSnapshot Snapshot() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
#endif
