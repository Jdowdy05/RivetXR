#include "room_scene.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using quest_newton::RoomScene;
namespace kin = quest_newton::kinematics;
void Check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
void Near(float actual, float expected, const char* reason) { Check(std::abs(actual-expected)<1e-4F,reason); }
template<class T> T Handle(std::uintptr_t value) { return reinterpret_cast<T>(value); }
XrUuidEXT Uuid(unsigned char value) { XrUuidEXT id{}; id.data[0]=value; return id; }
struct Query { XrAsyncRequestIdFB id; std::vector<unsigned char> uuids; bool delivered=false; };
struct Fixture {
    std::vector<Query> queries;
    std::map<XrSpace,unsigned char> spaces;
    std::set<XrSpace> destroyed;
    std::uintptr_t next_space=100;
    XrAsyncRequestIdFB next_id=1, capture_id=0;
    XrTime time=1000000000;
    bool missing_table=false, invalid_bounds=false, unavailable_floor=false, ambiguous=false;
    bool room_overflow=false, container_overflow=false, collider_overflow=false;
    bool invisible_wall=false, upside_down=false;
    bool other_room_far=false, concave_floor=false;
    bool many_members=false;
    bool unavailable_other_floor=false, missing_last_member=false;
    bool other_floor_activation_failure=false, other_floor_activation_sync_failure=false;
    bool locatable=true;
    bool capture_available=true;
    unsigned char count_only_floor=2;
    bool zero_walls=false, filled_layout_failure=false, filled_floor_invalid=false, filled_count_change=false;
    bool excessive_layout_walls=false;
    bool require_other_label=false;
    unsigned layout_count_calls=0, layout_filled_calls=0;
    std::map<unsigned char,kin::Vec3> anchor_offsets;
    float floor_yaw=0;
    bool negate_rotations=false;
    std::vector<XrSpace> locate_calls;
    std::vector<std::pair<XrAsyncRequestIdFB,XrSpace>> activations;
} fixture;
kin::Pose World() { kin::Pose pose; pose.rotation={-.5F,.5F,.5F,.5F}; return pose; }
const auto session=Handle<XrSession>(2);
const auto stage=Handle<XrSpace>(3);
void Update(RoomScene& reader, bool tracked=true, bool permission=true) {
    reader.Update(fixture.time,stage,World(),kin::Pose{{0,1.5F,0}},tracked,permission);
}
XrResult XRAPI_PTR QuerySpaces(XrSession,const XrSpaceQueryInfoBaseHeaderFB* base,XrAsyncRequestIdFB* id) {
    const auto* info=reinterpret_cast<const XrSpaceQueryInfoFB*>(base);
    Query query{fixture.next_id++,{}};
    if(info->filter->type==XR_TYPE_SPACE_COMPONENT_FILTER_INFO_FB) {
        query.uuids={1}; if(fixture.ambiguous)query.uuids.push_back(6);
        if(fixture.room_overflow)for(unsigned char i=2;i<=17;++i)query.uuids.push_back(i);
    } else {
        const auto* filter=reinterpret_cast<const XrSpaceUuidFilterInfoFB*>(info->filter);
        Check(filter->uuidCount<=50,"UUID query batches respect the runtime's 50 UUID limit");
        for(std::uint32_t i=0;i<filter->uuidCount;++i)query.uuids.push_back(filter->uuids[i].data[0]);
        if(fixture.missing_table) std::erase(query.uuids,static_cast<unsigned char>(4));
        if(fixture.missing_last_member) std::erase(query.uuids,static_cast<unsigned char>(63));
    }
    Check(info->maxResultCount>=query.uuids.size(),"reader must bound query without truncating fixture");
    *id=query.id;fixture.queries.push_back(query);return XR_SUCCESS;
}
XrResult XRAPI_PTR Retrieve(XrSession,XrAsyncRequestIdFB id,XrSpaceQueryResultsFB* output) {
    auto it=std::find_if(fixture.queries.begin(),fixture.queries.end(),[id](const auto& q){return q.id==id;});
    if(it==fixture.queries.end())return XR_ERROR_VALIDATION_FAILURE;
    output->resultCountOutput=it->delivered?0:static_cast<std::uint32_t>(it->uuids.size());
    if(!output->resultCapacityInput)return XR_SUCCESS;
    if(output->resultCapacityInput<output->resultCountOutput)return XR_ERROR_SIZE_INSUFFICIENT;
    for(std::uint32_t i=0;i<output->resultCountOutput;++i) {
        const auto space=Handle<XrSpace>(fixture.next_space++);
        fixture.spaces[space]=it->uuids[i];output->results[i]={space,Uuid(it->uuids[i])};
    }
    it->delivered=true;return XR_SUCCESS;
}
XrResult XRAPI_PTR Components(XrSpace space,std::uint32_t capacity,std::uint32_t* count,XrSpaceComponentTypeFB* output) {
    const auto id=fixture.spaces.at(space);
    std::vector<XrSpaceComponentTypeFB> values;
    if(id==1||id==6)values={XR_SPACE_COMPONENT_TYPE_ROOM_LAYOUT_FB,XR_SPACE_COMPONENT_TYPE_SPACE_CONTAINER_FB};
    else values={XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB,XR_SPACE_COMPONENT_TYPE_BOUNDED_2D_FB,XR_SPACE_COMPONENT_TYPE_SEMANTIC_LABELS_FB};
    *count=static_cast<std::uint32_t>(values.size());
    if(capacity) {if(capacity<*count)return XR_ERROR_SIZE_INSUFFICIENT; std::copy(values.begin(),values.end(),output);}
    return XR_SUCCESS;
}
XrResult XRAPI_PTR ComponentStatus(XrSpace space,XrSpaceComponentTypeFB kind,XrSpaceComponentStatusFB* status) {
    status->enabled=(kind!=XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB||fixture.locatable)?XR_TRUE:XR_FALSE;
    if(kind==XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB&&fixture.spaces.at(space)==7&&
       (fixture.other_floor_activation_failure||fixture.other_floor_activation_sync_failure))status->enabled=XR_FALSE;
    status->changePending=XR_FALSE;return XR_SUCCESS;
}
XrResult XRAPI_PTR Enable(XrSpace space,const XrSpaceComponentStatusSetInfoFB*,XrAsyncRequestIdFB* id) {
    if(fixture.other_floor_activation_sync_failure&&fixture.spaces.at(space)==7)return XR_ERROR_RUNTIME_FAILURE;
    *id=fixture.next_id++;fixture.activations.emplace_back(*id,space);return XR_SUCCESS;
}
XrResult XRAPI_PTR Layout(XrSession,XrSpace space,XrRoomLayoutFB* layout) {
    const auto count=fixture.excessive_layout_walls?65U:fixture.zero_walls?0U:1U;
    layout->wallUuidCountOutput=count;
    if(!layout->wallUuidCapacityInput) {
        ++fixture.layout_count_calls;
        // OpenXR explicitly leaves floor/ceiling UUIDs unspecified for a size query.
        layout->floorUuid=fixture.count_only_floor?Uuid(fixture.count_only_floor):XrUuidEXT{};
        return XR_SUCCESS;
    }
    ++fixture.layout_filled_calls;
    Check(layout->wallUuids!=nullptr&&layout->wallUuidCapacityInput<=64,"filled layout uses a bounded nonnull wall buffer");
    if(fixture.filled_layout_failure)return XR_ERROR_RUNTIME_FAILURE;
    layout->floorUuid=fixture.filled_floor_invalid?XrUuidEXT{}:Uuid(fixture.spaces.at(space)==6?7:2);
    if(fixture.filled_count_change)layout->wallUuidCountOutput=count?0U:1U;
    if(layout->wallUuidCapacityInput<layout->wallUuidCountOutput)return XR_ERROR_SIZE_INSUFFICIENT;
    for(std::uint32_t i=0;i<layout->wallUuidCountOutput;++i)layout->wallUuids[i]=Uuid(static_cast<unsigned char>(3+i));
    return XR_SUCCESS;
}
XrResult XRAPI_PTR Container(XrSession,XrSpace,XrSpaceContainerFB* output) {
    output->uuidCountOutput=fixture.container_overflow?129:fixture.collider_overflow?68:fixture.many_members?60:4;
    if(output->uuidCapacityInput) {
        for(unsigned char i=0;i<4;++i)output->uuids[i]=Uuid(static_cast<unsigned char>(i+2));
        if(fixture.collider_overflow||fixture.many_members)
            for(unsigned char i=4;i<output->uuidCountOutput;++i)output->uuids[i]=Uuid(static_cast<unsigned char>(i+4));
    }
    return XR_SUCCESS;
}
XrResult XRAPI_PTR Bounds(XrSession,XrSpace space,XrRect2Df* rect) {
    const auto id=fixture.spaces.at(space);
    *rect=(id==2||id==7)?XrRect2Df{{-2,-3},{4,6}}:XrRect2Df{{1,2},{2,4}};
    if(fixture.invalid_bounds&&id==4)rect->extent.width=-1;return XR_SUCCESS;
}
XrResult XRAPI_PTR Boundary(XrSession,XrSpace,XrBoundary2DFB* output) {
    output->vertexCountOutput=fixture.concave_floor?6:4;
    if(output->vertexCapacityInput) {
        constexpr std::array<XrVector2f,4> points{{{-2,-3},{2,-3},{2,3},{-2,3}}};
        constexpr std::array<XrVector2f,6> concave{{{-2,-3},{2,-3},{2,-1},{-1,-1},{-1,3},{-2,3}}};
        if(fixture.concave_floor)std::copy(concave.begin(),concave.end(),output->vertices);
        else std::copy(points.begin(),points.end(),output->vertices);
    }
    return XR_SUCCESS;
}
XrResult XRAPI_PTR Labels(XrSession,XrSpace space,XrSemanticLabelsFB* output) {
    if(fixture.require_other_label) {
        const auto* support=static_cast<const XrSemanticLabelsSupportInfoFB*>(output->next);
        if(!support||support->type!=XR_TYPE_SEMANTIC_LABELS_SUPPORT_INFO_FB||!support->recognizedLabels)return XR_ERROR_VALIDATION_FAILURE;
        const auto recognized=std::string(",")+support->recognizedLabels+",";
        if(recognized.find(",OTHER,")==std::string::npos)return XR_ERROR_VALIDATION_FAILURE;
    }
    const auto id=fixture.spaces.at(space);
    const std::string label=(id==2||id==7)?"FLOOR":id==3?(fixture.invisible_wall?"INVISIBLE_WALL_FACE":"WALL_FACE"):(id==4||id>=8)?"TABLE":fixture.require_other_label?"OTHER":"COUCH";
    output->bufferCountOutput=static_cast<std::uint32_t>(label.size()+1);
    if(output->bufferCapacityInput)std::copy(label.c_str(),label.c_str()+label.size()+1,output->buffer);
    return XR_SUCCESS;
}
XrResult XRAPI_PTR Capture(XrSession,const XrSceneCaptureRequestInfoFB*,XrAsyncRequestIdFB* id) {
    *id=fixture.next_id++;fixture.capture_id=*id;return XR_SUCCESS;
}
void Complete(RoomScene& reader,XrAsyncRequestIdFB id) {
    XrEventDataSpaceQueryResultsAvailableFB available{XR_TYPE_EVENT_DATA_SPACE_QUERY_RESULTS_AVAILABLE_FB};available.requestId=id;
    reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&available));
    XrEventDataSpaceQueryCompleteFB complete{XR_TYPE_EVENT_DATA_SPACE_QUERY_COMPLETE_FB};complete.requestId=id;complete.result=XR_SUCCESS;
    reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&complete));
}
void Acquire(RoomScene& reader) {
    for(unsigned i=0;i<8&&!reader.Ready();++i) {
        Update(reader);
        const auto queries=fixture.queries;
        for(const auto& query:queries)if(!query.delivered)Complete(reader,query.id);
    }
    Update(reader);
}
void Init(RoomScene& reader) { Check(reader.Init(Handle<XrInstance>(1),session),"reader dispatch initializes"); reader.SetEnabled(true); }
} // namespace

extern "C" XRAPI_ATTR XrResult XRAPI_CALL xrGetInstanceProcAddr(XrInstance,const char* name,PFN_xrVoidFunction* function) {
    if (std::strcmp(name,"xrRequestSceneCaptureFB")==0 && !fixture.capture_available) {
        *function=nullptr;return XR_ERROR_FUNCTION_UNSUPPORTED;
    }
#define FN(api, value) if(std::strcmp(name,api)==0) {*function=reinterpret_cast<PFN_xrVoidFunction>(value);return XR_SUCCESS;}
    FN("xrQuerySpacesFB",QuerySpaces) FN("xrRetrieveSpaceQueryResultsFB",Retrieve)
    FN("xrEnumerateSpaceSupportedComponentsFB",Components) FN("xrGetSpaceComponentStatusFB",ComponentStatus)
    FN("xrSetSpaceComponentStatusFB",Enable) FN("xrGetSpaceRoomLayoutFB",Layout)
    FN("xrGetSpaceContainerFB",Container) FN("xrGetSpaceBoundingBox2DFB",Bounds)
    FN("xrGetSpaceBoundary2DFB",Boundary) FN("xrGetSpaceSemanticLabelsFB",Labels)
    FN("xrRequestSceneCaptureFB",Capture)
#undef FN
    *function=nullptr;return XR_ERROR_FUNCTION_UNSUPPORTED;
}
extern "C" XRAPI_ATTR XrResult XRAPI_CALL xrDestroySpace(XrSpace space) {
    Check(fixture.spaces.contains(space)&&!fixture.destroyed.contains(space),"destroy only an owned space once");
    fixture.destroyed.insert(space);return XR_SUCCESS;
}
extern "C" XRAPI_ATTR XrResult XRAPI_CALL xrLocateSpace(XrSpace space,XrSpace,XrTime,XrSpaceLocation* output) {
    Check(!fixture.destroyed.contains(space),"never locate a destroyed anchor");
    fixture.locate_calls.push_back(space);
    const auto id=fixture.spaces.at(space);
    output->locationFlags=XR_SPACE_LOCATION_POSITION_VALID_BIT|XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    if(fixture.unavailable_floor&&(id==2||id==7))output->locationFlags=0;
    if(fixture.unavailable_other_floor&&id==7)output->locationFlags=0;
    output->pose={{0,0,0,1},{0,0,0}};
    if(id==2||id==4||id==7||id>=8)output->pose.orientation={fixture.upside_down?.70710678F:-.70710678F,0,0,.70710678F};
    if(id==4)output->pose.position={.25F,1,.5F};
    if(id==7&&fixture.other_room_far)output->pose.position={20,0,0};
    if((id==2||id==7)&&fixture.floor_yaw!=0) {
        const kin::Pose plane{{},{output->pose.orientation.x,output->pose.orientation.y,output->pose.orientation.z,output->pose.orientation.w}};
        const kin::Pose yaw{{},{0,std::sin(fixture.floor_yaw*.5F),0,std::cos(fixture.floor_yaw*.5F)}};
        const auto rotated=kin::Compose(yaw,plane).rotation;
        output->pose.orientation={rotated[0],rotated[1],rotated[2],rotated[3]};
    }
    if(const auto offset=fixture.anchor_offsets.find(id);offset!=fixture.anchor_offsets.end()) {
        output->pose.position.x+=offset->second[0];output->pose.position.y+=offset->second[1];output->pose.position.z+=offset->second[2];
    }
    if(fixture.negate_rotations) {
        output->pose.orientation.x=-output->pose.orientation.x;output->pose.orientation.y=-output->pose.orientation.y;
        output->pose.orientation.z=-output->pose.orientation.z;output->pose.orientation.w=-output->pose.orientation.w;
    }
    return XR_SUCCESS;
}
int main() {
    try {
        for(const auto unspecified_floor:{0,99}) {
            fixture={};fixture.count_only_floor=static_cast<unsigned char>(unspecified_floor);
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready(),"count-only unspecified floor UUID must not reject the filled valid layout");
            Check(fixture.layout_count_calls==1&&fixture.layout_filled_calls==1,"layout uses count query followed by a filled query");
            reader.Shutdown();Check(fixture.destroyed.size()==fixture.spaces.size(),"unspecified size-query UUID never changes owned-space cleanup");
        }
        fixture={};fixture.zero_walls=true;fixture.count_only_floor=0;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready()&&fixture.layout_filled_calls==1,"zero-wall layout still needs nonzero capacity to obtain its floor UUID");
            reader.Shutdown();
        }
        for(unsigned failure=0;failure<4;++failure) {
            fixture={};fixture.filled_layout_failure=failure==0;fixture.filled_floor_invalid=failure==1;
            fixture.filled_count_change=failure>=2;fixture.zero_walls=failure==3;
            RoomScene reader;Init(reader);Acquire(reader);
            Check(!reader.Ready()&&!reader.Snapshot().enabled&&reader.Snapshot().colliders.empty(),
                  "filled layout failure, invalid floor or changed count cannot publish geometry");
            Check(fixture.layout_filled_calls==1,"reject only after the filled layout query");
            reader.Shutdown();Check(fixture.destroyed.size()==fixture.spaces.size(),"rejected filled layout releases all owned spaces");
        }
        fixture={};fixture.excessive_layout_walls=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(!reader.Ready()&&fixture.layout_filled_calls==0,"oversized wall count is rejected before allocating a filled buffer");
            reader.Shutdown();Check(fixture.destroyed.size()==fixture.spaces.size(),"oversized layout releases its owned space");
        }
        fixture={};fixture.require_other_label=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().colliders.size()==3,
                  "semantic support must include OTHER while unsupported objects remain ignored");
            reader.Shutdown();
        }
        fixture={};
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready(),"complete uniquely selected room becomes ready");
            const auto snapshot=reader.Snapshot();Check(snapshot.enabled&&snapshot.colliders.size()==3,"only floor wall and table slabs");
            const auto& floor=snapshot.colliders[0];Near(floor.world_from_collider.position[2],-.025F,"floor extends below its plane");
            const auto table=std::find_if(snapshot.colliders.begin(),snapshot.colliders.end(),[](const auto& item){return item.kind==quest_newton::RoomSurfaceKind::Table;});
            Near(table->world_from_collider.position[0],3.5F,"table includes rectangle offset and stage to world mapping");
            Near(table->world_from_collider.position[1],-2.25F,"table includes rectangle centre in local X");
            Near(table->world_from_collider.position[2],.975F,"table slab extends below measured top");
            Update(reader,false);Check(!reader.Ready(),"head tracking loss pauses ready state");
            Update(reader);Check(reader.Ready()&&reader.Snapshot().revision==snapshot.revision,"tracking recovery keeps frozen geometry");
            auto moved_world=World();moved_world.position={1,0,0};reader.Update(1000000000,stage,moved_world,kin::Pose{{0,1.5F,0}},true,true);
            Check(!reader.Ready(),"world registration changes invalidate the old room");
            reader.SetEnabled(false);Check(!reader.Snapshot().enabled&&reader.Snapshot().colliders.empty(),"disable publishes ground restoration");
            const auto revision=reader.Snapshot().revision;reader.Shutdown();Init(reader);Check(reader.Snapshot().revision>revision,"revision survives session lifecycle");
            reader.Shutdown();Check(fixture.destroyed.size()==fixture.spaces.size(),"shutdown releases every loaded anchor");
        }
        fixture={};
        {
            RoomScene reader;Init(reader);Update(reader);const auto old=fixture.queries.back().id;reader.Refresh();Complete(reader,old);
            Check(!reader.Ready()&&fixture.destroyed.size()==fixture.spaces.size(),"late query results drain without publishing stale room");
            Acquire(reader);Check(reader.Ready(),"refresh starts clean query after old completion");
            Update(reader,true,false);Check(!reader.Ready(),"permission revocation pauses");
        }
        fixture={};fixture.many_members=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().colliders.size()==59,"all query batches contribute to one complete collider snapshot");
            Check(fixture.queries.size()==4,"room and floor queries plus two member batches");
        }
        fixture={};fixture.many_members=true;fixture.missing_last_member=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(!reader.Ready()&&!reader.Snapshot().enabled,"incomplete later batch never publishes an earlier partial batch");
        }
        for(unsigned mode=0;mode<7;++mode) {
            fixture={};fixture.missing_table=mode==0;fixture.invalid_bounds=mode==1;fixture.ambiguous=mode==2;
            fixture.room_overflow=mode==3;fixture.container_overflow=mode==4;fixture.collider_overflow=mode==5;fixture.unavailable_floor=mode==6;
            RoomScene reader;Init(reader);Acquire(reader);Check(!reader.Ready(),"partial malformed or ambiguous rooms never publish");
            if(mode==0) {
                Update(reader,false);Update(reader);
                Check(reader.Status().find("incomplete")!=std::string::npos,"tracking recovery restores actionable acquisition failure");
            }
        }
        fixture={};fixture.invisible_wall=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().colliders.size()==2,"unsupported invisible wall is deliberately excluded");
        }
        fixture={};fixture.upside_down=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);Check(reader.Ready(),"downward captured normals are normalized upward");
            const auto floor=std::find_if(reader.Snapshot().colliders.begin(),reader.Snapshot().colliders.end(),[](const auto& item){return item.kind==quest_newton::RoomSurfaceKind::Floor;});
            Near(floor->world_from_collider.position[2],-.025F,"inverted normal does not extrude above floor");
        }
        fixture={};fixture.concave_floor=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(!reader.Ready(),"head inside floor rectangle but outside polygon cannot select that room");
        }
        fixture={};fixture.ambiguous=true;fixture.other_room_far=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().colliders.size()==3,"a unique containing room excludes the other room geometry");
            Check(std::find(fixture.queries.back().uuids.begin(),fixture.queries.back().uuids.end(),7)==fixture.queries.back().uuids.end(),
                  "member query only loads the selected room");
        }
        fixture={};fixture.ambiguous=true;fixture.unavailable_other_floor=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().colliders.size()==3,"an inaccessible other saved room does not block a complete containing room");
        }
        fixture={};fixture.ambiguous=true;fixture.other_floor_activation_failure=true;
        {
            RoomScene reader;Init(reader);Update(reader);Complete(reader,fixture.queries.back().id);
            Complete(reader,fixture.queries.back().id);Update(reader);
            const auto activation=fixture.activations.front();
            XrEventDataSpaceSetStatusCompleteFB event{XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB};
            event.requestId=activation.first;event.result=XR_ERROR_RUNTIME_FAILURE;event.space=activation.second;
            event.uuid=Uuid(7);event.componentType=XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB;event.enabled=XR_FALSE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Acquire(reader);
            Check(reader.Ready(),"failed activation of an unselected floor cannot abort selected room loading");
        }
        fixture={};fixture.ambiguous=true;fixture.other_floor_activation_sync_failure=true;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            Check(reader.Ready(),"synchronous activation failure makes only the inaccessible candidate unavailable");
        }
        fixture={};fixture.locatable=false;
        {
            RoomScene reader;Init(reader);Acquire(reader);Check(!reader.Ready()&&!fixture.activations.empty(),"locatable activation waits for completion");
            const auto activation=fixture.activations.front();
            XrEventDataSpaceSetStatusCompleteFB event{XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB};
            event.requestId=activation.first+100;event.result=XR_SUCCESS;event.space=activation.second;
            event.uuid=Uuid(fixture.spaces.at(activation.second));event.componentType=XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB;event.enabled=XR_TRUE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Update(reader);Check(!reader.Ready(),"wrong activation request cannot unlock room");
            event.requestId=activation.first;event.uuid=Uuid(99);
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Update(reader);Check(!reader.Ready(),"wrong activation UUID cannot unlock room");
            event.uuid=Uuid(fixture.spaces.at(activation.second));reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));
            Acquire(reader);
            const auto activations=fixture.activations;
            for(const auto& item:activations) {
                event.requestId=item.first;event.space=item.second;event.uuid=Uuid(fixture.spaces.at(item.second));
                reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));
            }
            Update(reader);Check(reader.Ready(),"matching activation events enable every required surface");
        }
        fixture={};fixture.locatable=false;
        {
            RoomScene reader;Init(reader);Acquire(reader);
            const auto floor=fixture.activations.front();
            XrEventDataSpaceSetStatusCompleteFB event{XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB};
            event.requestId=floor.first;event.result=XR_SUCCESS;event.space=floor.second;event.uuid=Uuid(2);
            event.componentType=XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB;event.enabled=XR_TRUE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Acquire(reader);
            const auto member=std::find_if(fixture.activations.begin(),fixture.activations.end(),[](const auto& value){return fixture.spaces.at(value.second)==4;});
            Check(member!=fixture.activations.end(),"selected table requires activation");
            event.requestId=member->first;event.space=member->second;event.uuid=Uuid(4);event.result=XR_ERROR_RUNTIME_FAILURE;event.enabled=XR_FALSE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Update(reader);
            Check(!reader.Ready()&&reader.Status().find("could not become locatable")!=std::string::npos,
                  "selected room member activation failure remains fatal");
        }
        fixture={};
        {
            RoomScene reader;Check(reader.Init(Handle<XrInstance>(1),session),"init for permission check");
            Update(reader);Check(fixture.queries.empty(),"default disabled never queries rooms");
            reader.SetEnabled(true);Update(reader,true,false);Check(fixture.queries.empty(),"missing permission never queries rooms");
            reader.RequestCapture();Update(reader,true,false);Check(!fixture.capture_id,"scan waits for permission");
            Update(reader);Check(fixture.capture_id!=0,"explicit pending scan starts after permission grant");
        }
        fixture={};
        {
            RoomScene reader;Init(reader);Acquire(reader);reader.RequestCapture();Update(reader);
            Check(fixture.capture_id&&!reader.Ready(),"explicit capture invalidates previous scene");
            XrEventDataSceneCaptureCompleteFB event{XR_TYPE_EVENT_DATA_SCENE_CAPTURE_COMPLETE_FB};event.requestId=fixture.capture_id+1;event.result=XR_SUCCESS;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Update(reader);Check(!reader.Ready(),"unrelated capture completion ignored");
            event.requestId=fixture.capture_id;reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Acquire(reader);Check(reader.Ready(),"owned capture completion reacquires room");
        }
        fixture={};
        {
            RoomScene reader;Init(reader);Update(reader);const auto pending_query=fixture.queries.back().id;
            reader.RequestCapture();reader.SetEnabled(false);Complete(reader,pending_query);Update(reader);
            Check(!fixture.capture_id&&!reader.Snapshot().enabled,
                  "turning collisions off discards a scan queued behind an unfinished query");
        }
        fixture={};
        {
            RoomScene reader;Init(reader);Acquire(reader);const auto before=fixture.queries.size();
            XrEventDataReferenceSpaceChangePending event{XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING};
            event.session=session;event.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_STAGE;event.changeTime=3000000000;event.poseValid=XR_TRUE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));reader.Refresh();Update(reader);
            Check(!reader.Ready()&&fixture.queries.size()==before,"Refresh cannot acquire before a pending STAGE change becomes effective");
            fixture.time=4000000000;Acquire(reader);Check(reader.Ready(),"pending Refresh acquires in the new STAGE frame after changeTime");
        }
        fixture={};
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Update(reader);
            const auto pending_query=fixture.queries.back().id;
            Check(reader.RequestCapture(),"scan behind a pending query is accepted as queued");
            Update(reader);
            Check(!fixture.capture_id&&reader.Status().find("queued")!=std::string::npos,
                  "queued scan must not claim the OS scan is in progress");
            now+=std::chrono::seconds(16);Update(reader);
            Check(!fixture.capture_id&&reader.Status().find("stalled")!=std::string::npos,
                  "stale pending query still reports a bounded stall and cancels queued scan");
            const auto revision=reader.Snapshot().revision;
            now+=std::chrono::seconds(16);Update(reader);reader.Refresh();Update(reader);
            Check(fixture.queries.size()==1&&!fixture.capture_id&&reader.Status().find("stalled")!=std::string::npos,
                  "Refresh cannot hide or bypass the still-owned stalled query");
            Check(reader.Snapshot().revision==revision+1,"stall report does not invalidate every frame");
            Check(!reader.RequestCapture(),"scan rejected while a timed-out query still blocks it");
            Complete(reader,pending_query);Update(reader);
            Check(!fixture.capture_id,"late old completion must not surprise-start the cancelled scan");
            Acquire(reader);Check(reader.Ready(),"explicit Refresh resumes querying after the stale terminal event");
        }
        fixture={};
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Update(reader);reader.Refresh();
            now+=std::chrono::seconds(16);Update(reader);
            Check(reader.Status().find("stalled")!=std::string::npos&&fixture.queries.size()==1,
                  "Refresh-staled query receives the same bounded stall feedback");
            reader.SetEnabled(false);Update(reader);
            Check(reader.Status()=="Room collisions off","Off remains usable with a stalled query");
        }
        fixture={};fixture.capture_available=false;
        {
            RoomScene reader;Check(!reader.RequestCapture(),"uninitialized scan is rejected");
            Check(reader.Init(Handle<XrInstance>(1),session),"room reading does not require optional scan dispatch");
            Check(!reader.RequestCapture(),"disabled scan is rejected");
            reader.SetEnabled(true);Acquire(reader);Check(reader.Ready(),"room reading works without scan support");
            Check(!reader.RequestCapture()&&reader.Status().find("unavailable")!=std::string::npos,
                  "unsupported scan returns failure for caller-owned persistent feedback");
            Update(reader);Check(reader.Ready()&&!fixture.capture_id,"unsupported scan does not discard valid room physics");
        }
        fixture={};
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            Check(reader.RequestCapture(),"supported scan accepted");Update(reader);
            const auto request_id=fixture.capture_id;Check(request_id!=0,"OS scan actually started");
            now+=std::chrono::seconds(60);reader.SetEnabled(false);Update(reader);
            reader.SetEnabled(true);Check(reader.RequestCapture(),"active capture remains owned across Off/On");Update(reader);
            Check(fixture.capture_id==request_id&&reader.Status()=="Room scan in progress",
                  "already-started scan is not cancelled or duplicated by query timeout handling");
            XrEventDataSceneCaptureCompleteFB event{XR_TYPE_EVENT_DATA_SCENE_CAPTURE_COMPLETE_FB};
            event.requestId=request_id;event.result=XR_SUCCESS;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Acquire(reader);
            Check(reader.Ready(),"matching OS completion releases capture ownership and reloads the room");
        }
        fixture={};
        {
            using namespace std::chrono_literals;
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            Check(fixture.spaces.size()-fixture.destroyed.size()==3,
                  "complete room retains only supported applied anchors for monitoring");
            fixture.locate_calls.clear();const auto queries=fixture.queries.size();
            now+=499ms;Update(reader);Check(fixture.locate_calls.empty(),"monitor does not poll before its periodic sweep");
            now+=1ms;Update(reader);Check(fixture.locate_calls.size()==3,"all three applied anchors are checked at the sweep deadline");
            Check(reader.Ready()&&quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen&&fixture.queries.size()==queries,
                  "healthy monitoring preserves room geometry and does not issue new queries");
            fixture.negate_rotations=true;fixture.anchor_offsets[5]={20,0,0};
            now+=500ms;Update(reader);
            Check(reader.Ready()&&quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "quaternion sign changes and unsupported object movement do not invalidate the room");
            fixture.anchor_offsets[4]={.019F,0,0};now+=500ms;Update(reader);
            Check(reader.Ready(),"subthreshold anchor translation remains usable");
            fixture.anchor_offsets[4]={.021F,0,0};now+=500ms;Update(reader);
            Check(!reader.Ready()&&reader.Status().find("alignment changed")!=std::string::npos,
                  "more than two centimetres of corner drift latches refresh-required status");
            Check(quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "health failure preserves the exact applied geometry and revision");
            Check(fixture.spaces.size()==fixture.destroyed.size(),"latched health failure releases monitoring handles");
            fixture.anchor_offsets.clear();fixture.negate_rotations=false;now+=500ms;Update(reader);
            Check(!reader.Ready()&&fixture.queries.size()==queries,"later valid poses cannot silently recover a drifted room");
            Update(reader,true,false);Update(reader,true,true);
            Check(!reader.Ready()&&fixture.queries.size()==queries&&quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "permission recovery cannot clear an existing room-health latch");
            const auto revision=reader.Snapshot().revision;reader.Refresh();Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().revision>revision,"explicit Refresh reacquires a complete monitored room");
            reader.SetEnabled(false);
            Check(fixture.spaces.size()==fixture.destroyed.size(),"Off releases every retained applied anchor");
        }
        fixture={};
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            fixture.floor_yaw=.0174532925F;now+=std::chrono::milliseconds(500);Update(reader);
            Check(!reader.Ready()&&reader.Status().find("alignment changed")!=std::string::npos&&
                  quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "large floor rotation invalidates by corner displacement even with an unchanged anchor centre");
            fixture.floor_yaw=0;Check(reader.RequestCapture(),"explicit Scan may recover a latched room health failure");Update(reader);
            XrEventDataSceneCaptureCompleteFB event{XR_TYPE_EVENT_DATA_SCENE_CAPTURE_COMPLETE_FB};
            event.requestId=fixture.capture_id;event.result=XR_SUCCESS;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Acquire(reader);
            Check(reader.Ready(),"successful requested Scan establishes a fresh monitoring baseline");
        }
        fixture={};
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            fixture.unavailable_floor=true;now+=std::chrono::milliseconds(500);Update(reader);
            Check(!reader.Ready()&&reader.Status().find("anchor unavailable")!=std::string::npos&&
                  quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "selected-anchor localization loss pauses without discarding applied geometry");
            fixture.unavailable_floor=false;now+=std::chrono::milliseconds(100);Update(reader);
            Check(!reader.Ready(),"localization recovery requires an explicit Refresh after loss");
        }
        fixture={};
        {
            using namespace std::chrono_literals;
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            now+=1s;Check(reader.Ready(),"exactly one second of verification age is within the maximum");
            now+=1ns;Check(!reader.Ready(),"expired verification cannot remain Ready between XR updates");Update(reader,false);
            Update(reader,true);
            Check(!reader.Ready()&&reader.Status().find("verification expired")!=std::string::npos&&
                  quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "staleness latches even across head tracking loss and preserves the snapshot");
        }
        fixture={};fixture.locatable=false;
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto floor=fixture.activations.front();
            XrEventDataSpaceSetStatusCompleteFB event{XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB};
            event.requestId=floor.first;event.result=XR_SUCCESS;event.space=floor.second;event.uuid=Uuid(2);
            event.componentType=XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB;event.enabled=XR_TRUE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));Acquire(reader);
            fixture.locatable=true;Update(reader);Check(reader.Ready(),"polled activation can finish before its event arrives");
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            const auto member=std::find_if(fixture.activations.begin(),fixture.activations.end(),[](const auto& value){return fixture.spaces.at(value.second)==4;});
            Check(member!=fixture.activations.end(),"retained table has a pending activation completion");
            event.requestId=member->first;event.space=member->second;event.uuid=Uuid(4);event.result=XR_ERROR_RUNTIME_FAILURE;event.enabled=XR_FALSE;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));
            Check(!reader.Ready()&&quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "late retained-anchor activation failure preserves applied geometry and latches health loss");
        }
        fixture={};fixture.many_members=true;
        {
            using namespace std::chrono_literals;
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            Check(fixture.spaces.size()-fixture.destroyed.size()==59,"large room retains exactly its supported applied anchors");
            fixture.locate_calls.clear();now+=500ms;
            for(unsigned frame=0;frame<15;++frame) {
                const auto before=fixture.locate_calls.size();Update(reader);
                Check(fixture.locate_calls.size()-before<=4,"monitor consumes at most four anchor probes per XR update");
                now+=10ms;
            }
            Check(reader.Ready()&&fixture.locate_calls.size()==59,"a bounded sweep covers every applied surface once");
            now=std::chrono::steady_clock::time_point{}+999ms;Update(reader);
            Check(fixture.locate_calls.size()==59,"completed sweep does not restart before the next 500ms interval");
            now+=1ms;Update(reader);Check(fixture.locate_calls.size()==63,"next sweep begins at its scheduled interval");
        }
        fixture={};fixture.many_members=true;
        {
            using namespace std::chrono_literals;
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            now+=500ms;Update(reader);now+=501ms;Update(reader);
            Check(!reader.Ready()&&reader.Status().find("verification expired")!=std::string::npos,
                  "partial sweep cannot renew verification for unchecked anchors");
        }
        fixture={};
        {
            using namespace std::chrono_literals;
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            const auto queries=fixture.queries.size();
            fixture.anchor_offsets[4]={.021F,0,0};now+=500ms;Update(reader);
            Check(!reader.Ready(),"drift precondition for reference recovery latch regression");
            XrEventDataReferenceSpaceChangePending event{XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING};
            event.session=session;event.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_STAGE;event.changeTime=3000000000;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));reader.Invalidate();
            Check(!reader.Refresh(false),"automatic STAGE recovery cannot clear a latched health failure");
            Update(reader,true,false);Update(reader,true,true);
            XrEventDataEventsLost lost{XR_TYPE_EVENT_DATA_EVENTS_LOST};lost.lostEventCount=1;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&lost));
            Check(!reader.Refresh(false),"a subsequent generic XR error cannot erase explicit-refresh intent");
            Update(reader);
            Check(!reader.Ready()&&fixture.queries.size()==queries&&quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "reference and permission events preserve the stale applied snapshot until user recovery");
            Check(reader.Refresh(),"explicit user Refresh can clear the health latch");Update(reader);
            Check(fixture.queries.size()==queries,"explicit Refresh still respects the pending STAGE changeTime");
            fixture.time=4000000000;Acquire(reader);Check(reader.Ready(),"explicit Refresh reacquires after the new STAGE becomes effective");
            fixture.anchor_offsets[4]={.045F,0,0};now+=500ms;Update(reader);Check(!reader.Ready(),"second drift creates a new latch");
            reader.SetEnabled(false);reader.SetEnabled(true);Acquire(reader);
            Check(reader.Ready(),"explicit Off-On begins a new acquisition despite a previous health latch");
        }
        fixture={};
        {
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto revision=reader.Snapshot().revision,queries=fixture.queries.size();
            XrEventDataReferenceSpaceChangePending event{XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING};
            event.session=session;event.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_STAGE;event.changeTime=3000000000;
            reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));reader.Invalidate();
            Check(reader.Refresh(false),"healthy room permits automatic STAGE recovery");Update(reader);
            Check(!reader.Ready()&&fixture.queries.size()==queries,"automatic healthy recovery waits for changeTime");
            fixture.time=4000000000;Acquire(reader);
            Check(reader.Ready()&&reader.Snapshot().revision>revision,"healthy automatic recovery remains functional");
        }
        for(const unsigned recovery:{0U,1U,2U,3U}) {
            fixture={};
            auto now=std::chrono::steady_clock::time_point{};
            RoomScene reader([&now]{return now;});Init(reader);Acquire(reader);
            const auto frozen=quest_newton::RoomEnvironmentJson(reader.Snapshot());
            now+=std::chrono::milliseconds(1001);Check(!reader.Ready(),"expiry exists before the next Update");
            if(recovery==1) {
                XrEventDataReferenceSpaceChangePending event{XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING};
                event.session=session;event.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_STAGE;event.changeTime=3000000000;
                reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));reader.Invalidate();
            } else if(recovery==2) {
                Update(reader,true,false);Update(reader,true,true);
            } else if(recovery==3) {
                XrEventDataEventsLost event{XR_TYPE_EVENT_DATA_EVENTS_LOST};event.lostEventCount=1;
                reader.OnEvent(reinterpret_cast<XrEventDataBaseHeader*>(&event));
            }
            Check(!reader.Refresh(false)&&quest_newton::RoomEnvironmentJson(reader.Snapshot())==frozen,
                  "automatic recovery cannot outrun detection of expired anchor verification");
        }
        std::cout<<"room reader lifecycle, geometry and monitoring checks passed\n";return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
