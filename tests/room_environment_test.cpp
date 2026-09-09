#include "room_environment.h"
#include "scene_snapshot.h"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace { void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);} }
int main() {
    using namespace quest_newton;
    try {
        RoomEnvironment off;
        std::string error;
        Check(ValidateRoomEnvironment(off,error),"default off is valid");
        Check(RoomEnvironmentJson(off)=="{\"version\":1,\"revision\":0,\"enabled\":false,\"colliders\":[]}","off wire format");
        RoomEnvironment room;room.revision=2;room.enabled=true;
        RoomCollider floor;floor.half_extents={2,2,.025F};floor.world_from_collider.position={1,2,-.025F};
        room.colliders.push_back(floor);
        Check(ValidateRoomEnvironment(room,error),"bounded floor valid");
        Check(RoomEnvironmentJson(room).find("\"kind\":\"floor\"")!=std::string::npos,"kind serialized");
        auto bad=room;bad.colliders[0].kind=RoomSurfaceKind::Table;
        Check(!ValidateRoomEnvironment(bad,error),"enabled room needs floor");
        bad=room;bad.colliders[0].world_from_collider.rotation={0,0,0,2};
        Check(!ValidateRoomEnvironment(bad,error),"nonunit rotation rejected");
        bad=room;bad.colliders[0].half_extents[1]=std::numeric_limits<float>::quiet_NaN();
        Check(!ValidateRoomEnvironment(bad,error),"nonfinite bounds rejected");
        bad=room;bad.enabled=false;
        Check(!ValidateRoomEnvironment(bad,error),"off cannot hide colliders");
        Check(!RoomPhysicsBlocked(false,false,false,off,nullptr),"ordinary off mode never needs room");
        Check(RoomPhysicsBlocked(true,false,true,room,nullptr),"pending query pauses");
        Check(RoomPhysicsBlocked(true,true,true,room,nullptr),"ready data must be applied first");
        Check(!RoomPhysicsBlocked(true,true,true,room,&room),"matching applied room runs");
        Check(RoomPhysicsBlocked(true,true,false,room,&room),"stale registration pauses");
        bad=room;++bad.revision;
        Check(RoomPhysicsBlocked(true,true,true,bad,&room),"refresh waits for application");
        Check(RoomPhysicsBlocked(false,false,true,off,&room),"disable waits until old colliders removed");
        Check(!RoomPhysicsBlocked(false,false,true,off,&off),"disabled room resumes");
        SceneSnapshot scene;scene.room=std::make_shared<const RoomEnvironment>(room);
        SceneMailbox mailbox;mailbox.Publish(scene);const auto previous=mailbox.Read();
        scene.room=std::make_shared<const RoomEnvironment>(off);mailbox.Publish(scene);
        Check(previous.room->enabled && previous.room->revision==2,"old publication owns immutable room");
        Check(!mailbox.Read().room->enabled,"new publication carries applied off batch");
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
