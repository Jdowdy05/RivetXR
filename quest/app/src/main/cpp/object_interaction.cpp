#include "object_interaction.h"
#include "scene_snapshot.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace quest_newton {
namespace kin=kinematics;
namespace {
using Vector=std::array<double,3>;
struct Box {Vector center,half;std::array<Vector,3> axes;};
constexpr double kOverlapTolerance=1e-5; // metres; much smaller than support clearance
double Dot(const Vector& a,const Vector& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
Vector Cross(const Vector& a,const Vector& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
bool MakeBox(const kin::Pose& pose,const kin::Vec3& half,Box& box){
    kin::Pose normalized;if(!kin::NormalizePose(pose,normalized))return false;
    for(std::size_t axis=0;axis<3;++axis){
        if(!std::isfinite(half[axis]) || half[axis]<=0)return false;
        box.center[axis]=normalized.position[axis];box.half[axis]=half[axis];
    }
    // Form orthonormal axes in double precision; do not amplify float quaternion
    // roundoff when testing near-parallel edge cross-products.
    double x=normalized.rotation[0],y=normalized.rotation[1],z=normalized.rotation[2],w=normalized.rotation[3];
    const double scale=1/std::sqrt(x*x+y*y+z*z+w*w);x*=scale;y*=scale;z*=scale;w*=scale;
    box.axes={Vector{1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)},
              Vector{2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)},
              Vector{2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)}};
    return true;
}
bool Penetrates(const Box& a,const Box& b){
    Vector offset{};for(std::size_t axis=0;axis<3;++axis)offset[axis]=b.center[axis]-a.center[axis];
    const auto separated=[&](Vector axis){
        const double squared=Dot(axis,axis);
        if(squared<1e-20)return false; // Parallel edges add no independent SAT axis.
        const double inverse_length=1/std::sqrt(squared);for(auto& value:axis)value*=inverse_length;
        double radius=0;
        for(std::size_t i=0;i<3;++i)radius+=a.half[i]*std::abs(Dot(a.axes[i],axis))+b.half[i]*std::abs(Dot(b.axes[i],axis));
        return std::abs(Dot(offset,axis))>=radius-kOverlapTolerance;
    };
    for(const auto& axis:a.axes)if(separated(axis))return false;
    for(const auto& axis:b.axes)if(separated(axis))return false;
    for(const auto& first:a.axes)for(const auto& second:b.axes)if(separated(Cross(first,second)))return false;
    return true;
}
bool ClearVolume(const kin::Pose& pose,const kin::Vec3& half,const RoomEnvironment& room,
                 const SceneSnapshot* scene,std::optional<std::uint32_t> ignored_body){
    Box candidate;if(!MakeBox(pose,half,candidate))return false;
    if(room.enabled)for(const auto& surface:room.colliders){
        Box other;if(!MakeBox(surface.world_from_collider,surface.half_extents,other) || Penetrates(candidate,other))return false;
    }
    else {
        double vertical_radius=0;
        for(std::size_t axis=0;axis<3;++axis)vertical_radius+=candidate.half[axis]*std::abs(candidate.axes[axis][2]);
        if(candidate.center[2]-vertical_radius < -kOverlapTolerance)return false;
    }
    if(scene)for(const auto& object:scene->objects){
        if(ignored_body && object.body_index==*ignored_body)continue;
        if(object.kind!=1 || object.body_index>=scene->bodies.size())return false;
        Box other;if(!MakeBox(scene->bodies[object.body_index],object.half_extents,other) || Penetrates(candidate,other))return false;
    }
    return true;
}
} // namespace
bool CubePlacementClear(const kin::Pose& pose,const kin::Vec3& half,const RoomEnvironment& room,
                        const SceneSnapshot* scene,std::optional<std::uint32_t> ignored_body){
    for(float value:pose.position)if(!std::isfinite(value) || std::abs(value)>100.F)return false;
    double norm=0;
    for(float value:pose.rotation){if(!std::isfinite(value))return false;norm+=static_cast<double>(value)*value;}
    if(std::abs(std::sqrt(norm)-1.)>1e-3)return false;
    for(float value:half)if(!std::isfinite(value) || value<.005F || value>.1F)return false;
    if(room.colliders.size()>kMaxRoomColliders || (scene && scene->objects.size()>64))return false;
    if(ignored_body && (!scene || *ignored_body>=scene->bodies.size()))return false;
    return ClearVolume(pose,half,room,scene,ignored_body);
}
bool PlacementReleaseObserved(InputValue trigger){return trigger.active && std::isfinite(trigger.value) && trigger.value>=0 && trigger.value<.2F;}
InputValue ObjectPlacementMenu(bool placing,const SimSettings& settings,const ActionValues& actions,const InputValues& inputs){
    const auto menu=static_cast<std::size_t>(SimAction::Menu);
    if(!placing || settings.bindings[menu]!=InputId::RightTrigger)return actions[menu];
    // The physical right index confirms placement. Keep cancellation possible
    // when the normal Menu action has been remapped onto that same input.
    const auto value=inputs[static_cast<std::size_t>(InputId::LeftMenu)];
    if(!value.active || !std::isfinite(value.value) || value.value<0 || value.value>1)return {};
    return {value.value>.5F?1.F:0.F,true};
}
std::string ObjectEditJson(const ObjectEdit& edit){
    const char* op=edit.kind==ObjectEditKind::Spawn?"spawn":edit.kind==ObjectEditKind::Move?"move":
        edit.kind==ObjectEditKind::Reset?"reset":"remove";
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(9);
    out<<"{\"version\":1,\"op\":\""<<op<<"\",\"id\":"<<edit.id;
    if(edit.kind==ObjectEditKind::Spawn || edit.kind==ObjectEditKind::Move){
        out<<",\"pose\":[";bool first=true;
        for(float value:edit.pose.position){out<<(first?"":",")<<value;first=false;}
        for(float value:edit.pose.rotation)out<<','<<value;
        out<<']';
    }
    if(edit.kind==ObjectEditKind::Spawn)out<<",\"half_extents\":["<<edit.half_extents[0]<<','<<edit.half_extents[1]<<','<<edit.half_extents[2]<<']';
    out<<'}';return out.str();
}
std::optional<kin::Pose> CubePlacement(const kin::Pose& aim,const kin::Pose& frame,const RoomEnvironment& room,const kin::Vec3& half,
                                      const SceneSnapshot* scene,std::optional<std::uint32_t> ignored_body){
    kin::Pose normalized_aim,normalized_frame;
    if(!kin::NormalizePose(aim,normalized_aim) || !kin::NormalizePose(frame,normalized_frame))return {};
    for(float extent:half)if(!std::isfinite(extent) || extent<.005F || extent>.1F)return {};
    const auto world_aim=kin::Compose(kin::Inverse(normalized_frame),normalized_aim);
    const auto end=kin::Compose(world_aim,{{0,0,-1},{0,0,0,1}}).position;
    kin::Vec3 direction{};for(std::size_t i=0;i<3;++i)direction[i]=end[i]-world_aim.position[i];
    float nearest=3.001F;std::optional<kin::Pose> result;
    const auto try_surface=[&](const RoomCollider& surface){
        kin::Pose pose;if(!kin::NormalizePose(surface.world_from_collider,pose))return;
        const auto inverse=kin::Inverse(pose);
        const auto origin=kin::Compose(inverse,world_aim).position;
        const auto local_end=kin::Compose(inverse,{end,{0,0,0,1}}).position;
        kin::Vec3 delta{};for(std::size_t i=0;i<3;++i)delta[i]=local_end[i]-origin[i];
        // Every slab occludes the ray, including walls and a tabletop on which
        // the whole cube cannot fit. Never place through it onto a farther floor.
        float enter=0,leave=3.F;std::size_t face=3;float face_sign=0;
        for(std::size_t axis=0;axis<3;++axis){
            if(std::abs(delta[axis])<1e-6F){if(std::abs(origin[axis])>surface.half_extents[axis])return;continue;}
            float a=(-surface.half_extents[axis]-origin[axis])/delta[axis];
            float b=(surface.half_extents[axis]-origin[axis])/delta[axis];
            const float sign=delta[axis]<0?1.F:-1.F;if(a>b)std::swap(a,b);
            if(a>enter){enter=a;face=axis;face_sign=sign;}
            leave=std::min(leave,b);if(enter>leave)return;
        }
        if(leave<0 || enter>=nearest)return;
        nearest=enter;result.reset();
        if(enter<.05F || face!=2 || face_sign!=1 || surface.kind==RoomSurfaceKind::Wall)return;
        const auto normal_end=kin::Compose(pose,{{0,0,1},{0,0,0,1}}).position;
        kin::Vec3 normal{};for(std::size_t i=0;i<3;++i)normal[i]=normal_end[i]-pose.position[i];
        if(normal[2]<.5F || delta[2]>=-1e-6F)return;
        const float distance=enter;
        // Project a world-aligned box into the surface's axes before the fit test.
        kin::Vec3 extent{};
        for(std::size_t axis=0;axis<3;++axis){
            kin::Pose basis;basis.position[axis]=1;
            const auto rotated=kin::Compose({{},inverse.rotation},basis).position;
            for(std::size_t row=0;row<3;++row)extent[row]+=std::abs(rotated[row])*half[axis];
        }
        for(std::size_t axis=0;axis<2;++axis)
            if(std::abs(origin[axis]+delta[axis]*distance)+extent[axis]+.002F>surface.half_extents[axis])return;
        kin::Pose cube;
        for(std::size_t axis=0;axis<3;++axis)
            cube.position[axis]=world_aim.position[axis]+direction[axis]*distance+normal[axis]*(extent[2]+.002F);
        result=cube;
    };
    if(room.enabled){for(const auto& surface:room.colliders)try_surface(surface);}
    else try_surface({RoomSurfaceKind::Floor,{{0,0,-.025F},{0,0,0,1}},{100.F,100.F,.025F}});
    // Ray visibility and support fit are necessary but insufficient: a cube can
    // clip a wall (or another cube) which its centre ray never intersects.
    if(result && !CubePlacementClear(*result,half,room,scene,ignored_body))return {};
    return result;
}
void ObjectPlacement::Begin(ObjectEdit edit){edit_=edit;active_=true;released_=false;down_=true;preview_.reset();}
std::optional<ObjectEdit> ObjectPlacement::Update(std::optional<kin::Pose> candidate,bool tracked,bool trigger_active,float trigger,
    std::uint64_t reference,std::uint64_t generation,std::uint64_t revision,std::uint64_t suspension){
    if(!active_)return {};
    if(!tracked || !trigger_active || !std::isfinite(trigger) || trigger<0 || trigger>1 || reference!=edit_.reference ||
       generation!=edit_.settings_generation || revision!=edit_.room_revision || suspension!=edit_.suspension){Cancel();return {};}
    preview_=candidate;const bool down=trigger>.5F;
    if(trigger<.2F)released_=true;
    const bool confirm=down && !down_ && released_;down_=down;
    if(confirm){released_=false;if(candidate){auto command=edit_;command.pose=*candidate;Cancel();return command;}}
    return {};
}
} // namespace quest_newton
