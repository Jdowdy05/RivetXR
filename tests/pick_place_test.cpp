#include "object_interaction.h"
#include "scene_details.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void Check(bool v,const char* reason){if(!v)throw std::runtime_error(reason);}
template<class T>void Put(std::vector<std::byte>& out,std::size_t at,T v){std::memcpy(out.data()+at,&v,sizeof(v));}
}
int main(){
    using namespace quest_newton;
    try {
        SimSettings bindings;ActionValues actions{};InputValues raw{};
        bindings.bindings[static_cast<std::size_t>(SimAction::Menu)]=InputId::RightTrigger;
        bindings.bindings[static_cast<std::size_t>(SimAction::ArmEngage)]=InputId::LeftMenu;
        actions[static_cast<std::size_t>(SimAction::Menu)]={1,true};
        raw[static_cast<std::size_t>(InputId::LeftMenu)]={0,true};
        Check(ObjectPlacementMenu(false,bindings,actions,raw).value==1,"normal menu remap retained");
        Check(ObjectPlacementMenu(true,bindings,actions,raw).value==0,"placement confirmation cannot cancel itself through menu remap");
        raw[static_cast<std::size_t>(InputId::LeftMenu)]={1,true};
        Check(ObjectPlacementMenu(true,bindings,actions,raw).value==1,"physical left Menu remains a placement cancellation fallback");
        Check(!PlacementReleaseObserved({1,true}) && !PlacementReleaseObserved({0,false}),"held or inactive confirm cannot resume remapped robot controls");
        Check(PlacementReleaseObserved({0,true}) && !PlacementReleaseObserved({-.1F,true}),"only a valid physical confirm release ends input suppression");
        kinematics::Pose aim;aim.position={0,0,1}; // Local -Z points at world ground.
        RoomEnvironment room;
        auto hit=CubePlacement(aim,{},room,{.025F,.025F,.025F});
        Check(hit && std::abs(hit->position[2]-.027F)<1e-6F,"ground cube rests above surface");
        room.enabled=true;room.revision=3;
        room.colliders.push_back({RoomSurfaceKind::Table,{{0,0,.7F},{}},{.2F,.3F,.025F}});
        room.colliders[0].world_from_collider.rotation={0,0,0,1};
        hit=CubePlacement(aim,{},room,{.025F,.025F,.025F});
        Check(hit && std::abs(hit->position[2]-.752F)<1e-6F,"table top uses slab upper face");
        const kinematics::Pose stage_from_world{{.3F,1.F,-.7F},{-.70710678F,0,0,.70710678F}};
        const auto stage_aim=kinematics::Compose(stage_from_world,aim);
        const auto transformed=CubePlacement(stage_aim,stage_from_world,room,{.025F,.025F,.025F});
        Check(transformed && std::abs(transformed->position[2]-hit->position[2])<1e-5F,"placement converts STAGE Y-up into world Z-up");
        auto far_aim=aim;far_aim.position[2]=5;
        Check(!CubePlacement(far_aim,{},room,{.025F,.025F,.025F}),"placement ray range bounded");
        auto below=aim;below.position[2]=.5F;below.rotation={1,0,0,0};
        Check(!CubePlacement(below,{},room,{.025F,.025F,.025F}),"underside is not a supporting top face");
        aim.position[0]=.19F;Check(!CubePlacement(aim,{},room,{.025F,.025F,.025F}),"whole cube footprint must fit");
        room.colliders.push_back({RoomSurfaceKind::Floor,{{0,0,-.025F},{0,0,0,1}},{2,2,.025F}});
        Check(!CubePlacement(aim,{},room,{.025F,.025F,.025F}),"table edge cannot place through tabletop onto farther floor");
        aim.position[0]=0;room.colliders[0].kind=RoomSurfaceKind::Wall;
        Check(!CubePlacement(aim,{},room,{.025F,.025F,.025F}),"wall blocks placement onto farther floor");
        RoomEnvironment corner;corner.enabled=true;corner.revision=4;
        corner.colliders={
            {RoomSurfaceKind::Floor,{{0,0,-.025F},{0,0,0,1}},{1,1,.025F}},
            {RoomSurfaceKind::Wall,{{1,0,.75F},{0,.70710678F,0,.70710678F}},{.75F,1,.025F}}};
        kinematics::Pose corner_aim{{.96F,0,1.5F},{0,0,0,1}};
        Check(!CubePlacement(corner_aim,{},corner,{.025F,.025F,.025F}),
              "cube volume must not penetrate a neighboring wall that the centre ray misses");
        corner_aim.position[0]=.94F;
        Check(CubePlacement(corner_aim,{},corner,{.025F,.025F,.025F}).has_value(),"clear floor near wall remains usable");
        corner_aim.position[0]=.95F;
        Check(CubePlacement(corner_aim,{},corner,{.025F,.025F,.025F}).has_value(),"numerical wall touching is not penetration");
        corner_aim.position[0]=.9501F;
        Check(!CubePlacement(corner_aim,{},corner,{.025F,.025F,.025F}),"wall penetration beyond geometric epsilon rejected");
        const kinematics::Pose yaw{{},{0,0,std::sin(.2F),std::cos(.2F)}};
        auto rotated_corner=corner;
        for(auto& collider:rotated_corner.colliders)collider.world_from_collider=kinematics::Compose(yaw,collider.world_from_collider);
        corner_aim.position[0]=.96F;
        Check(!CubePlacement(kinematics::Compose(yaw,corner_aim),{},rotated_corner,{.025F,.025F,.025F}),
              "rotated neighboring wall also blocks overlapping cube volume");
        corner_aim.position[0]=.92F;
        Check(CubePlacement(kinematics::Compose(yaw,corner_aim),{},rotated_corner,{.025F,.025F,.025F}).has_value(),
              "rotated clear corner remains valid");

        RoomEnvironment slope;slope.enabled=true;slope.revision=5;
        slope.colliders={{RoomSurfaceKind::Floor,{{0,0,-.025F},{0,0,0,1}},{2,2,.025F}},
            {RoomSurfaceKind::Table,{{0,0,.7F},{std::sin(.1F),0,0,std::cos(.1F)}},{.4F,.4F,.025F}}};
        const auto sloped=CubePlacement({{0,0,1.5F},{0,0,0,1}}, {},slope,{.025F,.025F,.025F});
        Check(sloped.has_value(),"sloped support is not rejected by its own volume");
        float clearance=1;
        const auto surface_inverse=kinematics::Inverse(slope.colliders[1].world_from_collider);
        for(float x:{-.025F,.025F})for(float y:{-.025F,.025F})for(float z:{-.025F,.025F}){
            const auto corner_pose=kinematics::Compose(*sloped,{{x,y,z},{0,0,0,1}});
            clearance=std::min(clearance,kinematics::Compose(surface_inverse,corner_pose).position[2]-.025F);
        }
        Check(std::abs(clearance-.002F)<5e-6F,"sloped support keeps the full cube two millimetres clear");

        const kinematics::Pose ground_aim{{0,0,1},{0,0,0,1}};
        const RoomEnvironment off;
        SceneSnapshot occupied;occupied.bodies.resize(14);
        Check(CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied).has_value(),
              "robot poses alone do not create approximate object obstacles");
        occupied.bodies[12]={{0,0,.027F},{0,0,0,1}};
        occupied.objects.push_back({12,1,{.025F,.025F,.025F}});
        Check(!CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied),"existing user cube blocks overlapping spawn");
        Check(CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied,12).has_value(),
              "Move ignores exactly its own existing body");
        occupied.bodies[13]={{.04F,0,.05F},{0,0,0,1}};occupied.objects.push_back({13,1,{.05F,.05F,.05F}});
        Check(!CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied,12),
              "ignoring moving body does not ignore overlapping legacy cube");
        occupied.objects.resize(1);
        occupied.bodies[12]={{.055F,.055F,.027F},{0,0,.38268343F,.92387953F}};
        Check(CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied).has_value(),
              "rotated cube can be clear despite overlapping world AABBs");
        occupied.bodies[12].position={.04F,.04F,.027F};
        Check(!CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied),"rotated cube penetration rejected");
        occupied.bodies[12]={{.03F,.03F,.089F},{.20519567F,.30779351F,.10259784F,.92338052F}};
        Check(CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied).has_value(),
              "edge cross-product separating axis must prevent false rejection");
        occupied.bodies[12]={{.05F,0,.027F},{0,0,1e-7F,1}};
        Check(CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied).has_value(),"nearly parallel touching cubes remain valid");
        occupied.bodies[12].position[0]=.0499F;
        Check(!CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied),"nearly parallel overlapping cubes rejected");
        occupied.objects[0].body_index=99;
        Check(!CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied),"invalid supplied object geometry fails closed");
        occupied.objects[0].body_index=12;occupied.bodies[12]={{.2F,0,.027F},{0,0,0,1}};
        const auto pending_pose=CubePlacement(ground_aim,{},off,{.025F,.025F,.025F},&occupied);
        Check(pending_pose.has_value(),"preview starts clear of moving object");
        Check(CubePlacementClear(*pending_pose,{.025F,.025F,.025F},off,&occupied),"worker can recheck a previously clear pose");
        occupied.bodies[12].position[0]=.01F;
        Check(!CubePlacementClear(*pending_pose,{.025F,.025F,.025F},off,&occupied),"worker recheck catches object moved since preview");
        Check(CubePlacementClear(*pending_pose,{.025F,.025F,.025F},off,&occupied,12),"worker recheck preserves moving-body exemption");
        Check(!CubePlacementClear(*pending_pose,{.101F,.025F,.025F},off),"oversized final candidate rejected");
        Check(!CubePlacementClear({{101,0,.027F},{0,0,0,1}},{.025F,.025F,.025F},off),"out-of-world candidate rejected");
        Check(!CubePlacementClear({{0,0,.027F},{0,0,0,2}},{.025F,.025F,.025F},off),"nonunit final candidate rejected");
        Check(!CubePlacementClear({{0,0,.01F},{0,0,0,1}},{.025F,.025F,.025F},off),"ordinary ground penetration rejected at worker recheck");
        Check(!CubePlacementClear(*pending_pose,{.025F,.025F,.025F},off,&occupied,99),"invalid ignored body rejected");
        ObjectPlacement placement;ObjectEdit edit;edit.id=5;edit.reference=4;edit.settings_generation=2;edit.room_revision=3;
        placement.Begin(edit);
        Check(!placement.Update(hit,true,true,1,4,2,3),"held click cannot place");
        Check(!placement.Update(hit,true,true,0,4,2,3),"release arms placement");
        auto command=placement.Update(hit,true,true,1,4,2,3);
        Check(command && command->id==5 && !placement.Active(),"new click confirms exactly once");
        placement.Begin(edit);placement.Update(hit,true,true,0,4,2,3);
        Check(!placement.Update(hit,false,true,1,4,2,3) && !placement.Active(),"tracking loss cancels pending placement");
        placement.Begin(edit);placement.Update(hit,true,true,0,4,2,3);
        Check(!placement.Update(hit,true,true,1,5,2,3) && !placement.Active(),"new reference cancels pending placement");
        for(const auto epoch:{0,1}){
            placement.Begin(edit);placement.Update(hit,true,true,0,4,2,3);
            Check(!placement.Update(hit,true,true,1,4,epoch==0?3:2,epoch==1?4:3) && !placement.Active(),"settings and room changes cancel placement");
        }
        placement.Begin(edit);placement.Update(hit,true,true,0,4,2,3);
        Check(!placement.Update(hit,true,true,1,4,2,3,1) && !placement.Active(),"transient loss epoch cancels placement after tracking returns");
        placement.Begin(edit);placement.Update(hit,true,true,0,4,2,3);
        Check(!placement.Update({},true,true,1,4,2,3),"missing surface consumes click without placement");
        Check(!placement.Update(hit,true,true,1,4,2,3),"held click after invalid aim cannot place");

        SceneSnapshot scene;scene.model_generation=7;scene.step_index=9;scene.simulation_time=.045;scene.contact_count=2;
        scene.bodies.resize(13);scene.objects.push_back({12,1,{.025F,.025F,.025F}});
        std::vector<std::byte> bytes(48+24+36);std::memcpy(bytes.data(),"QDIA",4);
        Put<std::uint16_t>(bytes,4,1);Put<std::uint16_t>(bytes,6,48);
        Put<std::uint32_t>(bytes,8,7);Put<std::uint32_t>(bytes,12,13);Put<std::uint64_t>(bytes,16,9);
        Put<double>(bytes,24,.045);Put<std::uint32_t>(bytes,32,1);Put<std::uint32_t>(bytes,36,1);Put<std::uint32_t>(bytes,40,1);
        Put<std::uint32_t>(bytes,48,5);Put<std::uint32_t>(bytes,52,12);Put<std::uint32_t>(bytes,56,1);
        for(int i=0;i<3;++i)Put<float>(bytes,60+i*4,.025F);
        Put<std::int32_t>(bytes,72,12);Put<std::int32_t>(bytes,76,-1);Put<float>(bytes,100,1.F);Put<float>(bytes,104,2.F);
        SceneDetails details;std::string error;
        Check(DecodeSceneDetails(bytes,scene,details,error),"matching QDIA details accepted");
        Check(details.objects[0].id==5 && details.contacts[0].normal_force==2,"IDs and contact force decoded");
        auto signed_force=bytes;Put<float>(signed_force,104,-.25F);
        Check(DecodeSceneDetails(signed_force,scene,details,error) && details.contacts[0].normal_force==-.25F,"signed diagnostic force is not clipped");
        auto bad=bytes;Put<std::uint64_t>(bad,16,8);
        Check(!DecodeSceneDetails(bad,scene,details,error) && details.step_index==9,"stale sidecar rejected transactionally");
        bad=bytes;Put<std::uint32_t>(bad,52,0);Check(!DecodeSceneDetails(bad,scene,details,error),"object cannot bind robot");
        bad=bytes;Put<float>(bad,104,std::numeric_limits<float>::quiet_NaN());
        Check(!DecodeSceneDetails(bad,scene,details,error),"nonfinite diagnostic rejected");
        bad=bytes;bad.pop_back();Check(!DecodeSceneDetails(bad,scene,details,error),"truncated detail rejected");
        bad=bytes;Put<std::uint32_t>(bad,44,4);Check(!DecodeSceneDetails(bad,scene,details,error),"unknown diagnostic flags rejected");
        auto stale=bytes;stale.resize(48+24);Put<std::uint32_t>(stale,36,0);Put<std::uint32_t>(stale,40,0);Put<std::uint32_t>(stale,44,2);
        Check(DecodeSceneDetails(stale,scene,details,error) && !details.contacts_current && details.objects[0].id==5,
              "stale contacts retain identity mapping without claiming zero contacts");
        bad=bytes;Put<std::uint32_t>(bad,44,2);Check(!DecodeSceneDetails(bad,scene,details,error),"stale marker cannot contain contact forces");
        bad=bytes;Put<std::uint32_t>(bad,40,2);Check(!DecodeSceneDetails(bad,scene,details,error),"truncation cannot be hidden");
        SceneDetailsPoll polling;
        Check(polling.Due(7,false,0),"initial object map must load with overlays off");
        polling.Failed(0);
        Check(!polling.Due(7,false,.49) && polling.Due(7,false,.5),"failed initial map retries at bounded rate with overlays off");
        polling.Succeeded(7,.5);
        Check(!polling.Due(7,false,1.5) && polling.Due(7,true,1.5),"successful map stays cached; enabled contacts are refreshed");
        Check(polling.Due(8,false,.51),"a new model needs a new object map");
        SceneDetailsPoll cadence;cadence.Attempt(0,true);cadence.Succeeded(7,0);
        Check(cadence.Due(8,true,.05) && !cadence.ContactsDue(true,.05),"new object mapping cannot bypass contact rate cap");
        cadence.Attempt(.05,false);cadence.Succeeded(8,.05);cadence.Invalidate();
        Check(!cadence.ContactsDue(true,.099) && cadence.ContactsDue(true,.1),"generation changes preserve independent contact cadence");
        SceneDetailsPoll recreated;recreated.Attempt(0,true);recreated.Succeeded(2,0);
        Check(!recreated.Due(2,false,.05),"fixture begins with cached generation two");
        recreated.BeginSession();
        Check(recreated.Due(2,false,.05) && !recreated.ContactsDue(true,.05),"new Session reloads reused generation without bypassing contact cadence");
        recreated.Failed(.05);recreated.BeginSession();
        Check(recreated.Due(2,false,.06),"new Session does not inherit failed lookup cooldown");
        std::cout<<"pick-place placement and detail contracts passed\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
