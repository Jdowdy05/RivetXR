#include "remote_scene.h"
#include "remote_scene_renderer.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace quest_newton;
using Bytes=std::vector<std::byte>;
void Check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
template<class T>void Put(Bytes& data,std::size_t at,T value){
    static_assert(std::endian::native==std::endian::little);
    std::memcpy(data.data()+at,&value,sizeof(T));
}
Bytes Packet(std::uint32_t count=1){
    Bytes bytes(128+static_cast<std::size_t>(count)*16);
    std::memcpy(bytes.data(),"RSCN",4);Put<std::uint16_t>(bytes,4,1);Put<std::uint16_t>(bytes,6,128);
    for(std::size_t offset:{8U,16U,24U,32U})Put<std::uint64_t>(bytes,offset,1);
    Put<std::uint64_t>(bytes,40,count?1000000000:0);Put<std::uint64_t>(bytes,48,count?1040000000:0);
    Put<std::uint64_t>(bytes,56,1045000000);Put(bytes,64,count);
    Put<std::uint32_t>(bytes,68,31);Put<std::uint32_t>(bytes,72,count?31:0);
    Put<std::uint32_t>(bytes,76,kRemoteSceneSynthetic|(count?0:kRemoteScenePartial));
    Put(bytes,80,.01F);Put<std::uint32_t>(bytes,84,50);
    // Independent CRC oracles produced with Python zlib over <fffBBH> bytes.
    Put<std::uint32_t>(bytes,88,count==1?0xb2f1281e:count==50000?0xbe5916fb:0);
    Put(bytes,96,-1.F);Put(bytes,104,1.F);Put(bytes,120,1.F);
    for(std::uint32_t i=0;i<count;++i){const auto at=128+static_cast<std::size_t>(i)*16;
        Put(bytes,at,1.F);Put(bytes,at+4,-2.F);Put(bytes,at+8,3.5F);
        Put<std::uint8_t>(bytes,at+12,128);Put<std::uint8_t>(bytes,at+13,3);Put<std::uint16_t>(bytes,at+14,10);
    }
    return bytes;
}
void Reject(const Bytes& bytes){
    RemoteSceneFrame output;output.sequence=99;std::string error;
    Check(!DecodeRemoteScene(bytes,output,error)&&output.sequence==99&&!error.empty(),"invalid packet changed output or was accepted");
}
}
int main(int argc,char** argv){
    using namespace quest_newton;using namespace std::chrono_literals;
    try{
        if(argc==2){
            std::ifstream input(argv[1],std::ios::binary|std::ios::ate);
            Check(static_cast<bool>(input),"cannot open remote scene fixture");const auto length=input.tellg();
            Check(length>=0&&length<=static_cast<std::streamoff>(kRemoteSceneMaxPacketBytes),"remote scene fixture exceeds packet bound");
            Bytes bytes(static_cast<std::size_t>(length));input.seekg(0);
            input.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
            Check(static_cast<bool>(input)&&input.peek()==std::char_traits<char>::eof(),"remote scene fixture read failed or changed");RemoteSceneFrame decoded;std::string error;
            if(!DecodeRemoteScene(bytes,decoded,error))throw std::runtime_error(error);
            std::cout<<"RSCN fixture decoded: points="<<decoded.points.size()<<" cameras="<<decoded.contributing_mask<<" flags="<<decoded.flags<<'\n';return 0;
        }
        Check(argc==1,"expected no arguments or one RSCN fixture path");
        std::string error;RemoteSceneFrame frame;const auto valid=Packet();
        Check(DecodeRemoteScene(valid,frame,error),"exact RSCN v1 fixture must decode");
        Check(frame.Identity()==RemoteSceneIdentity{1,1,1}&&frame.sequence==1&&frame.points.size()==1,"identity/sequence/count decoded");
        Check(frame.points[0].position==kinematics::Vec3{1,-2,3.5F}&&frame.points[0].grayscale==128&&
              frame.points[0].support==3&&frame.points[0].radius_mm==10,"metric position, grayscale, support count and radius preserved");
        Check(frame.capture_start_ns==1000000000&&frame.capture_end_ns==1040000000&&frame.produced_ns==1045000000,
              "producer clock fields remain separate and unchanged");
        Check(frame.suggested_observer.position==kinematics::Vec3{-1,0,1},"observer pose decoded at the documented offset");
        auto no_cameras=Packet(0);Check(DecodeRemoteScene(no_cameras,frame,error)&&frame.points.empty()&&frame.contributing_mask==0,
              "no-camera heartbeat is a complete empty frame");
        auto zero_produced=no_cameras;Put<std::uint64_t>(zero_produced,56,0);Reject(zero_produced);
        auto partial=valid;Put<std::uint32_t>(partial,72,7);Put<std::uint32_t>(partial,76,kRemoteScenePartial|kRemoteSceneRecorded|kRemoteSceneSynthetic);
        Check(DecodeRemoteScene(partial,frame,error)&&frame.flags==14&&frame.expected_camera_mask==31&&frame.contributing_mask==7,
              "partial coverage and source flags are reported without reinterpreting confidence");
        const auto maximum=Packet(50000);Check(DecodeRemoteScene(maximum,frame,error)&&frame.points.size()==50000,"maximum point budget accepted");
        auto malformed=maximum;malformed.push_back(std::byte{});Reject(malformed);
        malformed=valid;malformed.pop_back();Reject(malformed);
        malformed=valid;malformed[128]^=std::byte{1};Reject(malformed);
        for(const auto offset:{0U,4U,6U,92U,124U}){malformed=valid;malformed[offset]^=std::byte{1};Reject(malformed);}
        for(const auto offset:{8U,16U,24U,32U}){malformed=valid;Put<std::uint64_t>(malformed,offset,0);Reject(malformed);}
        for(const auto offset:{68U,72U}){malformed=valid;Put<std::uint32_t>(malformed,offset,32);Reject(malformed);}
        malformed=valid;Put<std::uint32_t>(malformed,68,0);Reject(malformed);
        malformed=valid;Put<std::uint32_t>(malformed,76,16);Reject(malformed);
        malformed=valid;Put<std::uint32_t>(malformed,72,7);Reject(malformed);
        malformed=valid;Put<std::uint32_t>(malformed,76,kRemoteScenePartial);Reject(malformed);
        malformed=partial;Put<std::uint32_t>(malformed,72,1);Reject(malformed); // support=3 exceeds actual cameras
        malformed=no_cameras;Put<std::uint64_t>(malformed,40,1);Reject(malformed);
        malformed=valid;Put<std::uint32_t>(malformed,72,0);Put<std::uint32_t>(malformed,76,kRemoteScenePartial);Reject(malformed);
        for(float voxel:{0.F,.0009F,.101F,std::numeric_limits<float>::quiet_NaN()}){malformed=valid;Put(malformed,80,voxel);Reject(malformed);}
        for(std::uint32_t skew:{0U,101U}){malformed=valid;Put(malformed,84,skew);Reject(malformed);}
        malformed=valid;Put<std::uint64_t>(malformed,40,0);Reject(malformed);
        malformed=valid;Put<std::uint64_t>(malformed,48,999999999);Reject(malformed);
        malformed=valid;Put<std::uint64_t>(malformed,48,1050000001);Put<std::uint64_t>(malformed,56,1050000001);Reject(malformed);
        malformed=valid;Put<std::uint64_t>(malformed,56,1039999999);Reject(malformed);
        malformed=valid;Put<std::uint64_t>(malformed,56,3040000001);Reject(malformed);
        malformed=valid;Put(malformed,96,101.F);Reject(malformed);
        malformed=valid;Put(malformed,120,1.01F);Reject(malformed);
        malformed=valid;Put(malformed,120,0.F);Reject(malformed);
        // Correct CRCs ensure these reach point-semantic validation.
        for(const auto pair:{std::pair{0x7fc00000U,0x4ec54e44U},std::pair{0x42ca0000U,0xdca7198eU}}){
            malformed=valid;Put(malformed,128,pair.first);Put(malformed,88,pair.second);Reject(malformed);
        }
        for(const auto pair:{std::pair{0U,0xb0b79647U},std::pair{6U,0xb43aeaf5U}}){
            malformed=valid;Put<std::uint8_t>(malformed,141,static_cast<std::uint8_t>(pair.first));Put(malformed,88,pair.second);Reject(malformed);
        }
        for(const auto pair:{std::pair{0U,0x481ec094U},std::pair{51U,0xbc75a5a4U}}){
            malformed=valid;Put<std::uint16_t>(malformed,142,static_cast<std::uint16_t>(pair.first));Put(malformed,88,pair.second);Reject(malformed);
        }
        RemoteSceneMailbox mailbox;const auto generation=mailbox.Reset();const auto received=RemoteSceneTime{}+10s;
        Check(mailbox.Publish(valid,received,generation,error),"first complete frame publishes");
        const auto snapshot=mailbox.Read();std::weak_ptr<const RemoteSceneFrame> held=snapshot.frame;
        Check(snapshot.received_at==received&&snapshot.frame->produced_ns==1045000000&&snapshot.stream_generation==generation,
              "mailbox uses local receipt time independently of producer time");
        Check(RemoteSceneFresh(snapshot,received+500ms)&&!RemoteSceneFresh(snapshot,received+501ms)&&
              !RemoteSceneFresh(snapshot,received-1ms)&&RemoteSceneAgeMs(snapshot,received+17ms)==17,
              "freshness and age use only a valid local clock interval");
        Check(!mailbox.Publish(valid,received+1ms,generation,error)&&mailbox.Read().publication==snapshot.publication,
              "duplicate sequence does not replace the complete frame");
        auto next=valid;Put<std::uint64_t>(next,24,2);
        Check(mailbox.Publish(next,received+1ms,generation,error)&&snapshot.frame->sequence==1,"published frame remains immutable after replacement");
        Check(!mailbox.Publish(valid,received+2ms,generation,error),"backward same-identity frame rejected");
        auto epoch=valid;Put<std::uint64_t>(epoch,16,2);
        Check(mailbox.Publish(epoch,received+3ms,generation,error)&&mailbox.Read().frame->world_epoch==2,"new world identity permits sequence restart");
        Check(!mailbox.Publish(valid,received+4ms,generation,error),"return to a prior identity cannot replay an old sequence");
        auto return_to_a=valid;Put<std::uint64_t>(return_to_a,24,3);
        Check(mailbox.Publish(return_to_a,received+4ms,generation,error),"a later sequence may return to a prior identity");
        const auto coalesced=mailbox.Read();
        Check(coalesced.frame->Identity()==snapshot.frame->Identity()&&coalesced.identity_revision>snapshot.identity_revision,
              "coalesced A-B-A identity changes must retire an old inspection alignment");
        next=epoch;Put<std::uint64_t>(next,24,2);
        Check(!mailbox.Publish(next,received,generation,error),"local receipt time cannot move backwards");
        const auto reset_generation=mailbox.Reset();Check(!mailbox.Read().frame&&!RemoteSceneFresh(mailbox.Read(),received),"Reset retires the visible frame");
        Check(!mailbox.Publish(next,received+5ms,generation,error),"retired receiver generation cannot publish after Reset");
        Check(mailbox.Publish(valid,received+6ms,reset_generation,error),"new receiver generation can restart sequence");
        const auto heartbeat=Packet(0);auto heartbeat_next=heartbeat;Put<std::uint64_t>(heartbeat_next,24,2);
        Check(mailbox.Publish(heartbeat_next,received+7ms,reset_generation,error)&&mailbox.Read().frame->points.empty(),
              "no-camera heartbeat replaces geometry instead of retaining a false current cloud");
        const auto before_bad=mailbox.Read().publication;auto bad_crc=valid;bad_crc[128]^=std::byte{1};
        Check(!mailbox.Publish(bad_crc,received+8ms,reset_generation,error)&&mailbox.Read().publication==before_bad,
              "invalid frame does not refresh or replace the previous complete publication");
        for(std::uint64_t source=2;source<=64;++source){
            auto changed_source=valid;Put(changed_source,8,source);
            Check(mailbox.Publish(changed_source,received+std::chrono::milliseconds(100+source),reset_generation,error),"bounded identity history rejected an available slot");
        }
        auto too_many=valid;Put<std::uint64_t>(too_many,8,65);
        Check(!mailbox.Publish(too_many,received+200ms,reset_generation,error),"identity history must remain bounded");
        auto existing_identity=valid;Put<std::uint64_t>(existing_identity,8,64);Put<std::uint64_t>(existing_identity,24,2);
        Check(mailbox.Publish(existing_identity,received+201ms,reset_generation,error),"full history still accepts later frames of an existing identity");
        Check(!held.expired(),"consumer-held immutable snapshot keeps its own frame alive");
        Mat4 map;map.m[0]=map.m[5]=map.m[10]=map.m[15]=1;float pixel_scale=0;
        Check(RemoteSceneProjectionScale(map,2000,pixel_scale)&&pixel_scale==2000,"point diameter uses actual per-eye viewport height");
        map.m[0]=map.m[5]=map.m[10]=.25F;
        Check(RemoteSceneProjectionScale(map,2000,pixel_scale)&&pixel_scale==500,"quarter-scale inspection quarters point diameter");
        map.m[0]=map.m[5]=map.m[10]=2;
        Check(RemoteSceneProjectionScale(map,2000,pixel_scale)&&pixel_scale==4000,"double-scale inspection doubles point diameter");
        map.m[5]=1;Check(!RemoteSceneProjectionScale(map,2000,pixel_scale),"nonuniform scale cannot silently distort point radius");
        map.m[5]=2;map.m[0]=-2;Check(!RemoteSceneProjectionScale(map,2000,pixel_scale),"reflection is not an inspection rotation");
        map.m[0]=2;map.m[4]=1;Check(!RemoteSceneProjectionScale(map,2000,pixel_scale),"shear cannot silently distort point radius");
        map.m[4]=0;Check(!RemoteSceneProjectionScale(map,0,pixel_scale),"zero viewport rejected");
        map.m[15]=0;Check(!RemoteSceneProjectionScale(map,2000,pixel_scale),"non-affine transform rejected");
        std::cout<<"RSCN framing, CRC, geometry, coverage, clocks, identity and mailbox checks passed\n";
    }catch(const std::exception& exception){std::cerr<<exception.what()<<'\n';return 1;}
}
