#pragma once
#include "arm_renderer.h"
#include "scene_snapshot.h"
#include "scene_details.h"
#include <optional>
#include "Render/SurfaceRender.h"

namespace quest_newton {
struct CubeMarker {kinematics::Pose pose;kinematics::Vec3 half_extents{};};
class EnvironmentRenderer {
public:
    bool Init();
    void Shutdown();
    void Append(const SceneSnapshot& snapshot,const Mat4& stage_from_world,bool grid,bool room_overlay,
                std::vector<OVRFW::ovrDrawSurface>& surfaces) const;
    void AppendDiagnostics(const Mat4& stage_from_world,const std::optional<kinematics::Pose>& target,
        const std::optional<kinematics::Pose>& measured,const std::optional<kinematics::Pose>& preview,
        const kinematics::Vec3& half_extents,std::span<const ContactPoint> contacts,const std::optional<CubeMarker>& selected,
        std::vector<OVRFW::ovrDrawSurface>& surfaces) const;
private:
    OVRFW::GlProgram program_;
    OVRFW::ovrSurfaceDef grid_,box_,room_;
    OVRFW::ovrSurfaceDef target_,measured_,preview_,selected_,contact_,normal_;
    std::array<float,4> grid_color_{.09F,.14F,.16F,.18F};
    std::array<float,4> box_color_{.72F,.4F,.08F,.8F};
    std::array<float,4> room_color_{.06F,.55F,.45F,.65F};
    std::array<float,4> target_color_{.8F,.15F,.05F,.85F},measured_color_{.08F,.6F,.8F,.85F};
    std::array<float,4> preview_color_{.2F,.75F,.15F,.8F},contact_color_{.9F,.7F,.05F,.95F};
    std::array<float,4> selected_color_{.8F,.8F,.8F,.85F};
    bool ready_=false;
};
} // namespace quest_newton
