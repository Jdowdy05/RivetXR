#pragma once
#include "remote_scene.h"
#include <jni.h>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace quest_newton {
class RemoteSceneMailbox;
// Init/Start/Stop/Shutdown are serialized by the owning XR lifecycle thread.
// Connect/read, config/asset reads, decompression and parsing run on the receiver
// worker. The owner only closes I/O during Stop. Mailbox outlives this receiver.
class RemoteSceneReceiver {
public:
    explicit RemoteSceneReceiver(RemoteSceneMailbox& mailbox):mailbox_(mailbox){}
    ~RemoteSceneReceiver();
    RemoteSceneReceiver(const RemoteSceneReceiver&)=delete;
    RemoteSceneReceiver& operator=(const RemoteSceneReceiver&)=delete;
    bool Init(JavaVM* vm,JNIEnv* env,jobject activity,std::string& error);
    bool Start(JNIEnv* env,std::string& error);
    bool StartDemo(JNIEnv* env,std::string& error,RemoteSceneRepresentation representation=RemoteSceneRepresentation::Points,bool rgb=false);
    // Close Java I/O BEFORE join; use a live environment from this receiver's VM.
    // Stop retains the last frame for explicitly frozen/disconnected inspection.
    void Stop(JNIEnv* env);
    void Shutdown(JNIEnv* env);
    bool Running()const{return running_.load();}
    std::string Status()const;
private:
    bool StartImpl(JNIEnv* env,int demo_mode,std::string& error);
    void Run(jobject connection,std::uint64_t stream_generation,bool demo) noexcept;
    void SetStatus(std::string_view status) noexcept;
    RemoteSceneMailbox& mailbox_;
    JavaVM* vm_=nullptr;
    jobject context_=nullptr,connection_=nullptr;
    jclass connection_class_=nullptr;
    jmethodID constructor_=nullptr,connect_=nullptr,read_=nullptr,close_=nullptr;
    std::thread worker_;
    std::atomic<bool> stop_{true},running_{false};
    mutable std::mutex status_mutex_;
    std::string status_="Remote disconnected";
};
} // namespace quest_newton
