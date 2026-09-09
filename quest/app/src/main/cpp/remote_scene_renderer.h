#pragma once
#include "arm_renderer.h"
#include "remote_scene.h"
#include <memory>
#include <string>
#include <vector>

namespace OVRFW {struct ovrDrawSurface;}
namespace quest_newton {
// Per-eye pixel height times positive uniform map scale; false for invalid,
// sheared or reflected transforms. The shader applies radius and projection Y.
bool RemoteSceneProjectionScale(const Mat4& stage_from_map,float viewport_height,float& output);
// GL-thread owned; Init/Shutdown while the session's EGL context is current.
// Call Shutdown before destruction; the destructor never calls into a lost EGL context.
class RemoteSceneRenderer {
public:
    RemoteSceneRenderer();
    ~RemoteSceneRenderer();
    RemoteSceneRenderer(const RemoteSceneRenderer&)=delete;
    RemoteSceneRenderer& operator=(const RemoteSceneRenderer&)=delete;
    bool Init();
    // Upload only changed, decoder-validated publications; empty geometry clears
    // the scene. Two bounded GPU slots commit vertices/indices/atlas atomically.
    // Failure leaves the last GPU publication intact, and may be retried.
    bool Upload(const RemoteSceneSnapshot& snapshot);
    // viewport_height is per-eye pixels. Black background fills app projection
    // alpha only; system passthrough/guardian ownership remains with the caller.
    // Returns whether visible geometry was submitted. Live v3 observation TTL
    // includes time since receipt; recorded/synthetic inspection freezes that age.
    bool Append(const Mat4& stage_from_map,float viewport_height,bool stale,bool black_background,
                std::vector<OVRFW::ovrDrawSurface>& surfaces,RemoteSceneTime now=std::chrono::steady_clock::now());
    void Shutdown();
    bool Ready() const;
    const std::string& Error() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace quest_newton
