#include "remote_scene.h"
#include <bit>
#include <cstring>
#include <iostream>
#include <cmath>
#include <fstream>
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
void Crc(Bytes& data){
    std::uint32_t crc=0xffffffffU;
    for(std::size_t i=160;i<data.size();++i){crc^=std::to_integer<unsigned>(data[i]);
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((crc&1)?0xedb88320U:0U);
    }
    Put(data,88,~crc);
}
Bytes Raw(){
    Bytes b(160+160+18+9);std::memcpy(b.data(),"RSCN",4);
    Put<std::uint16_t>(b,4,2);Put<std::uint16_t>(b,6,160);
    for(std::size_t at:{8U,16U,24U,32U})Put<std::uint64_t>(b,at,1);
    for(std::size_t at:{40U,48U,56U})Put<std::uint64_t>(b,at,1000000000);
    Put<std::uint32_t>(b,68,1);Put<std::uint32_t>(b,72,1);
    Put<std::uint32_t>(b,76,kRemoteSceneSynthetic);Put<std::uint32_t>(b,84,50);Put(b,120,1.F);
    Put<std::uint32_t>(b,128,2);Put<std::uint32_t>(b,148,1);Put<std::uint32_t>(b,152,187);
    for(std::size_t at:{164U,166U,168U,170U})Put<std::uint16_t>(b,at,3);
    for(std::size_t at:{176U,180U,192U,196U})Put(b,at,10.F);
    for(std::size_t at:{184U,188U,200U,204U})Put(b,at,1.F);
    Put(b,208,.001F);
    for(std::size_t base:{212U,260U})for(std::size_t axis=0;axis<3;++axis)Put(b,base+axis*20,1.F);
    Put<std::uint64_t>(b,312,1000000000);
    for(std::size_t i=0;i<9;++i){Put<std::uint16_t>(b,320+i*2,1000);Put<std::uint8_t>(b,338+i,static_cast<std::uint8_t>(i*25));}
    Crc(b);return b;
}
Bytes Prepared(const Bytes& common,const RemoteSceneFrame& frame){
    Bytes b(common.begin(),common.begin()+160);
    const auto count=frame.vertices.size(),indices=frame.indices.size();
    b.resize(160+count*24+indices*2+frame.atlas.size());
    Put<std::uint32_t>(b,128,1);Put<std::uint32_t>(b,132,static_cast<std::uint32_t>(count));
    Put<std::uint32_t>(b,136,static_cast<std::uint32_t>(indices));Put(b,140,frame.atlas_width);Put(b,144,frame.atlas_height);
    Put<std::uint32_t>(b,152,static_cast<std::uint32_t>(b.size()-160));
    for(std::size_t i=0;i<count;++i)for(std::size_t axis=0;axis<3;++axis){
        Put(b,160+i*24+axis*4,frame.vertices[i].position[axis]);Put(b,172+i*24+axis*4,frame.vertices[i].uvq[axis]);
    }
    for(std::size_t i=0;i<indices;++i)Put(b,160+count*24+i*2,frame.indices[i]);
    for(std::size_t i=0;i<frame.atlas.size();++i)Put(b,160+count*24+indices*2+i,frame.atlas[i]);
    Crc(b);return b;
}
void Reject(const Bytes& bytes){
    RemoteSceneFrame output;output.sequence=99;output.vertices.push_back({{1,2,3},{1,2,3}});std::string error;
    Check(!DecodeRemoteScene(bytes,output,error)&&output.sequence==99&&output.vertices.size()==1&&!error.empty(),"invalid surface packet accepted or changed output");
}
void EqualGeometry(const RemoteSceneFrame& a,const RemoteSceneFrame& b){
    Check(a.indices==b.indices&&a.atlas==b.atlas&&a.atlas_width==b.atlas_width&&a.atlas_height==b.atlas_height&&a.vertices.size()==b.vertices.size(),
          "prepared and worker-built indices/atlas/vertex counts must agree exactly");
    for(std::size_t i=0;i<a.vertices.size();++i)for(std::size_t j=0;j<3;++j){
        Check(std::abs(a.vertices[i].position[j]-b.vertices[i].position[j])<=1e-6F,"prepared versus worker XYZ mismatch");
        Check(std::abs(a.vertices[i].uvq[j]-b.vertices[i].uvq[j])<=1e-6F,"prepared versus worker projective texture coordinate mismatch");
    }
}
Bytes Read(const char* path){
    std::ifstream input(path,std::ios::binary|std::ios::ate);Check(static_cast<bool>(input),"cannot open surface fixture");
    const auto length=input.tellg();Check(length>=0&&length<=static_cast<std::streamoff>(kRemoteSceneMaxPacketBytes),"surface fixture size outside bounds");
    Bytes bytes(static_cast<std::size_t>(length));input.seekg(0);input.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    Check(static_cast<bool>(input)&&input.peek()==std::char_traits<char>::eof(),"surface fixture read failed");return bytes;
}
}
int main(int argc,char** argv){try{
    RemoteSceneFrame frame;std::string error;
    if(argc==3){RemoteSceneFrame raw,prepared;
        Check(DecodeRemoteScene(Read(argv[1]),prepared,error),error.c_str());
        Check(DecodeRemoteScene(Read(argv[2]),raw,error),error.c_str());
        Check(prepared.representation==RemoteSceneRepresentation::Prepared&&raw.representation==RemoteSceneRepresentation::Unprepared,"fixture modes incorrect");
        EqualGeometry(prepared,raw);std::cout<<"Cross-language surface fixture matches: vertices="<<raw.vertices.size()<<" triangles="<<raw.indices.size()/3<<'\n';return 0;
    }
    Check(argc==1,"expected zero arguments or prepared/unprepared fixture paths");
    const bool decoded=DecodeRemoteScene(Raw(),frame,error);
    if(!decoded)std::cerr<<error<<'\n';
    Check(decoded,"unprepared calibrated RSCN v2 frame must decode");
    Check(frame.representation==RemoteSceneRepresentation::Unprepared&&frame.vertices.size()==9&&frame.indices.size()==24&&frame.HasGeometry()&&frame.points.empty(),"3x3 planar depth must produce eight textured triangles");
    Check(frame.atlas_width==3&&frame.atlas_height==3&&frame.atlas[8]==200,"raw Y8 image copied to atlas");
    Check(frame.indices[0]==0&&frame.indices[1]==3&&frame.indices[2]==1,"surface triangles face the observing camera");
    Check(std::abs(frame.vertices[0].position[0]+.1F)<1e-6F&&std::abs(frame.vertices[0].uvq[0]/frame.vertices[0].uvq[2]-1.F/6)<1e-6F,
          "depth intrinsics and pixel centers define metric XYZ and projective UV");
    const auto raw=Raw(),prepared=Prepared(raw,frame);RemoteSceneFrame copy;
    Check(DecodeRemoteScene(prepared,copy,error)&&copy.representation==RemoteSceneRepresentation::Prepared,"prepared surface must decode");EqualGeometry(copy,frame);
    // Tiny but valid double-precision triangles can collapse after XYZ float32
    // serialization at distant world coordinates. Neither mode may emit them.
    auto collapsed=raw;
    for(std::size_t at:{176U,192U})Put(collapsed,at,2.1F);
    for(std::size_t at:{180U,196U})Put(collapsed,at,333333.33F);
    Put(collapsed,240,99.F);Crc(collapsed);
    Check(DecodeRemoteScene(collapsed,copy,error)&&!copy.HasGeometry(),"float32-collapsed triangles must be removed before publication");
    // Foreground samples outside the renderable map still occlude background
    // in the intensity camera. World clipping must follow visibility filtering.
    auto clipped=raw;Put(clipped,224,100.F);Put(clipped,192,.1F);Put(clipped,196,.1F);
    for(std::size_t y=0;y<3;++y)for(std::size_t x=0;x<2;++x)Put<std::uint16_t>(clipped,320+(y*3+x)*2,1020);
    Crc(clipped);
    Check(DecodeRemoteScene(clipped,copy,error)&&!copy.HasGeometry(),"out-of-map foreground must occlude in-map textured background");
    // Column orthogonality can fail while every row dot product lies within
    // tolerance. Both canonical checks are required for Python/native parity.
    auto nonrigid=raw;const auto c=std::cos(std::acos(-1.)/6),s=std::sqrt(1.0012);
    Put(nonrigid,212,static_cast<float>(c*s));Put(nonrigid,216,-.5F);
    Put(nonrigid,228,static_cast<float>(.5*s));Put(nonrigid,232,static_cast<float>(c));Crc(nonrigid);Reject(nonrigid);
    auto step=raw;
    for(std::size_t y=0;y<3;++y)Put<std::uint16_t>(step,320+(y*3+2)*2,2000);
    Crc(step);Check(DecodeRemoteScene(step,copy,error)&&copy.vertices.size()==6&&copy.indices.size()==12,"depth discontinuity must remain a hole rather than a stretched surface");
    auto hole=raw;Put<std::uint16_t>(hole,328,0);Crc(hole);
    Check(DecodeRemoteScene(hole,copy,error)&&copy.vertices.size()==6&&copy.indices.size()==6,"missing depth corner removes every attached triangle");
    auto invalid_depth=raw;for(std::size_t i=0;i<9;++i)Put<std::uint16_t>(invalid_depth,320+i*2,0);Crc(invalid_depth);
    Check(DecodeRemoteScene(invalid_depth,copy,error)&&!copy.HasGeometry()&&copy.atlas.empty()&&!copy.atlas_width&&copy.contributing_mask==1,
          "captured invalid depth publishes explicit empty geometry without claiming no cameras");
    auto heartbeat=Bytes(raw.begin(),raw.begin()+160);Put<std::uint32_t>(heartbeat,72,0);Put<std::uint32_t>(heartbeat,76,kRemoteSceneSynthetic|kRemoteScenePartial);
    Put<std::uint32_t>(heartbeat,148,0);Put<std::uint32_t>(heartbeat,152,0);Put<std::uint64_t>(heartbeat,40,0);Put<std::uint64_t>(heartbeat,48,0);Crc(heartbeat);
    Check(DecodeRemoteScene(heartbeat,copy,error)&&!copy.HasGeometry()&&copy.contributing_mask==0,"raw no-camera heartbeat is accepted");
    const auto prepared_empty=Prepared(heartbeat,copy);
    Check(DecodeRemoteScene(prepared_empty,copy,error)&&!copy.HasGeometry(),"prepared no-camera heartbeat is accepted");
    for(auto empty:{heartbeat,prepared_empty}){Put<std::uint64_t>(empty,56,0);Reject(empty);}
    // Descriptor preflight: every truncated prefix, length/count mismatch and
    // semantic mutation must reject atomically, even with an updated valid CRC.
    for(std::size_t size=0;size<raw.size();++size)Reject(Bytes(raw.begin(),raw.begin()+size));
    for(std::size_t at:{128U,132U,136U,140U,144U,148U,152U,156U,160U,172U,308U}){
        auto bad=raw;Put<std::uint32_t>(bad,at,0xffffffffU);Crc(bad);Reject(bad);
    }
    for(const auto [at,value]:{std::pair<std::size_t,unsigned>{164,1},{164,65},{166,1},{166,49},{168,1},{168,321},{170,1},{170,241}}){
        auto bad=raw;Put<std::uint16_t>(bad,at,static_cast<std::uint16_t>(value));Crc(bad);Reject(bad);
    }
    for(std::size_t at:{176U,180U,192U,196U,208U})for(float value:{0.F,-1.F,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}){
        auto bad=raw;Put(bad,at,value);Crc(bad);Reject(bad);
    }
    for(std::size_t at:{184U,188U,200U,204U,212U,228U,244U,260U,276U,292U}){
        auto bad=raw;Put(bad,at,std::numeric_limits<float>::quiet_NaN());Crc(bad);Reject(bad);
    }
    for(const auto [at,value]:{std::pair<std::size_t,float>{208,1.1F},{212,-1.F},{212,1.01F},{260,-1.F},{224,101.F},{272,-101.F}}){
        auto bad=raw;Put(bad,at,value);Crc(bad);Reject(bad);
    }
    for(std::uint64_t capture:{999999999ULL,1000000001ULL}){auto bad=raw;Put(bad,312,capture);Crc(bad);Reject(bad);}
    auto bad=raw;bad.push_back(std::byte{});Put<std::uint32_t>(bad,152,188);Crc(bad);Reject(bad);
    bad=raw;bad[320]^=std::byte{1};Reject(bad);
    bad=raw;Put(bad,80,-0.F);Reject(bad); // reserved field is exactly zero bits
    for(std::size_t at:{128U,132U,136U,140U,144U,148U,152U,156U}){bad=prepared;Put<std::uint32_t>(bad,at,0xffffffffU);Crc(bad);Reject(bad);}
    for(std::size_t at:{160U,172U,176U,180U}){bad=prepared;Put(bad,at,std::numeric_limits<float>::quiet_NaN());Crc(bad);Reject(bad);}
    for(const auto [at,value]:{std::pair<std::size_t,float>{160,101.F},{172,-.1F},{172,1.1F},{176,-.1F},{176,1.1F},{180,0.F},{180,1025.F}}){
        bad=prepared;Put(bad,at,value);Crc(bad);Reject(bad);
    }
    const auto index_at=160+frame.vertices.size()*24;
    bad=prepared;Put<std::uint16_t>(bad,index_at,9);Crc(bad);Reject(bad);
    bad=prepared;Put<std::uint16_t>(bad,index_at+2,0);Crc(bad);Reject(bad);
    bad=prepared;for(std::size_t axis=0;axis<3;++axis)Put(bad,160+3*24+axis*4,frame.vertices[0].position[axis]);Crc(bad);Reject(bad);
    bad=prepared;Put<std::uint32_t>(bad,136,23);Crc(bad);Reject(bad);
    bad=prepared;Put<std::uint32_t>(bad,72,0);Put<std::uint32_t>(bad,76,kRemoteSceneSynthetic|kRemoteScenePartial);Put<std::uint32_t>(bad,148,0);
    Put<std::uint64_t>(bad,40,0);Put<std::uint64_t>(bad,48,0);Reject(bad);
    // Maximum prepared packet stays bounded independently of wire counts.
    RemoteSceneFrame maximum=frame;maximum.vertices.resize(kRemoteSurfaceMaxVertices,frame.vertices[0]);
    maximum.indices.resize(kRemoteSurfaceMaxIndices);
    for(std::size_t i=0;i<maximum.indices.size();i+=3){maximum.indices[i]=0;maximum.indices[i+1]=3;maximum.indices[i+2]=1;}
    maximum.atlas_width=1600;maximum.atlas_height=240;maximum.atlas.resize(kRemoteSurfaceMaxAtlasPixels,127);
    const auto largest=Prepared(raw,maximum);
    Check(largest.size()==kRemoteSceneLegacyMaxPacketBytes&&DecodeRemoteScene(largest,copy,error)&&copy.vertices.size()==16384,"exact maximum prepared packet accepted");
    bad=largest;bad.push_back(std::byte{});Reject(bad);
    // A representation is a coordinate/alignment identity even when all the
    // producer's numeric IDs happen to match and readers coalesce transitions.
    using namespace std::chrono_literals;RemoteSceneMailbox mailbox;const auto generation=mailbox.Reset();const auto now=RemoteSceneTime{}+10s;
    Check(mailbox.Publish(raw,now,generation,error),"unprepared frame publishes");const auto first=mailbox.Read();
    Check(mailbox.Publish(prepared,now+1ms,generation,error),"prepared mode may restart its sequence");
    Check(!mailbox.Publish(raw,now+2ms,generation,error),"returning raw mode cannot replay its older sequence");
    auto next=raw;Put<std::uint64_t>(next,24,2);
    Check(mailbox.Publish(next,now+3ms,generation,error),"returning mode accepts a newer sequence");const auto last=mailbox.Read();
    Check(first.frame->Identity()==last.frame->Identity()&&last.identity_revision==first.identity_revision+2,"coalesced mode A-B-A invalidates alignment");
    Check(first.frame->representation==RemoteSceneRepresentation::Unprepared&&first.frame->vertices.size()==9,"immutable snapshot retains geometry and input representation");
    auto empty_next=heartbeat;Put<std::uint64_t>(empty_next,24,3);
    Check(mailbox.Publish(empty_next,now+4ms,generation,error)&&!mailbox.Read().frame->HasGeometry(),"no-camera surface publication retires prior visible geometry");
    std::cout<<"Remote surface checks passed\n";
}catch(const std::exception& exception){std::cerr<<exception.what()<<'\n';return 1;}}
