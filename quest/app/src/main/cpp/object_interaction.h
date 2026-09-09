#pragma once
#include "room_environment.h"
#include "sim_settings.h"
#include <optional>

namespace quest_newton {
struct SceneSnapshot;
enum class ObjectEditKind { Spawn,Move,Reset,Remove };
struct ObjectEdit {
    ObjectEditKind kind=ObjectEditKind::Spawn;
    std::uint32_t id=0;
    kinematics::Pose pose;
    kinematics::Vec3 half_extents{.025F,.025F,.025F};
    std::uint64_t reference=0,settings_generation=0,room_revision=0,suspension=0;
};
std::string ObjectEditJson(const ObjectEdit& edit);
InputValue ObjectPlacementMenu(bool placing,const SimSettings& settings,const ActionValues& actions,const InputValues& inputs);
bool PlacementReleaseObserved(InputValue trigger);
// Recheck an already selected world pose against current obstacle volumes.
// This does not select a support face or re-run the placement ray.
bool CubePlacementClear(const kinematics::Pose& world_from_cube,const kinematics::Vec3& half_extents,
    const RoomEnvironment& applied,const SceneSnapshot* existing_scene=nullptr,
    std::optional<std::uint32_t> ignored_body=std::nullopt);
// A gravity-aligned cube must fit on the support and not penetrate other slabs
// or supplied scene boxes. Touching within 1e-5m is numerical tolerance only.
// ignored_body must come from the same snapshot and is only for a moving cube.
std::optional<kinematics::Pose> CubePlacement(const kinematics::Pose& stage_from_aim,
    const kinematics::Pose& stage_from_world,const RoomEnvironment& applied,
    const kinematics::Vec3& half_extents,const SceneSnapshot* existing_scene=nullptr,
    std::optional<std::uint32_t> ignored_body=std::nullopt);
class ObjectPlacement {
public:
    void Begin(ObjectEdit edit);
    void Cancel(){active_=false;preview_.reset();}
    bool Active() const{return active_;}
    const auto& Preview() const{return preview_;}
    const auto& Edit() const{return edit_;}
    std::optional<ObjectEdit> Update(std::optional<kinematics::Pose> candidate,bool tracking_valid,
        bool trigger_active,float trigger,std::uint64_t reference,std::uint64_t settings_generation,
        std::uint64_t room_revision,std::uint64_t suspension=0);
private:
    ObjectEdit edit_;
    bool active_=false,released_=false,down_=true;
    std::optional<kinematics::Pose> preview_;
};
} // namespace quest_newton
