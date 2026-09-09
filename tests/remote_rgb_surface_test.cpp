#include "remote_scene.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace {
using namespace quest_newton;using Bytes=std::vector<std::byte>;
void Check(bool x,const char* why){if(!x)throw std::runtime_error(why);}
template<class T>void Put(Bytes& b,std::size_t at,T v){static_assert(std::endian::native==std::endian::little);std::memcpy(b.data()+at,&v,sizeof(T));}
void Crc(Bytes& b){std::uint32_t c=~0U;for(std::size_t i=224;i<b.size();++i){c^=std::to_integer<unsigned>(b[i]);for(int j=0;j<8;++j)c=(c>>1)^((c&1)?0xedb88320U:0U);}Put(b,88,~c);}
Bytes Raw(){
    Bytes b(224+208+18+27+1);std::memcpy(b.data(),"RSCN",4);Put<std::uint16_t>(b,4,3);Put<std::uint16_t>(b,6,224);
    for(std::size_t at:{8U,16U,24U,32U,160U})Put<std::uint64_t>(b,at,1);
    for(std::size_t at:{40U,48U,56U,168U})Put<std::uint64_t>(b,at,1000000000);
    Put<std::uint32_t>(b,68,1);Put<std::uint32_t>(b,72,1);Put<std::uint32_t>(b,84,50);Put(b,120,1.F);
    Put<std::uint32_t>(b,128,2);Put<std::uint32_t>(b,148,1);Put<std::uint32_t>(b,152,static_cast<std::uint32_t>(b.size()-224));
    Put<std::uint32_t>(b,176,1);Put<std::uint32_t>(b,184,30000);Put<std::uint32_t>(b,188,1);Put<std::uint32_t>(b,192,44);
    Put<std::uint32_t>(b,196,3);Put<std::uint32_t>(b,200,208);
    const std::size_t at=224;
    for(std::size_t i:{4U,6U,8U,10U})Put<std::uint16_t>(b,at+i,3);
    for(std::size_t i:{16U,20U,32U,36U})Put(b,at+i,10.F);
    for(std::size_t i:{24U,28U,40U,44U})Put(b,at+i,1.F);Put(b,at+48,.001F);
    for(std::size_t base:{52U,100U})for(std::size_t axis=0;axis<3;++axis)Put(b,at+base+axis*20,1.F);
    Put<std::uint64_t>(b,at+152,1000000000);Put<std::uint64_t>(b,at+160,1000000000);Put<std::uint64_t>(b,at+168,1);
    Put<std::uint32_t>(b,at+176,2);Put<std::uint32_t>(b,at+180,3);Put<std::uint32_t>(b,at+184,1);
    for(std::size_t i=0;i<9;++i){Put<std::uint16_t>(b,432+i*2,1000);Put<std::uint8_t>(b,450+i*3,255);}
    Put<std::uint8_t>(b,b.size()-1,255);Crc(b);return b;
}
Bytes Prepared(const Bytes& header,const RemoteSceneFrame& f){
    Bytes b(header.begin(),header.begin()+224);const auto n=f.observations.size();
    b.resize(224+n*80+f.rgb_vertices.size()*44+f.indices.size()*2+f.atlas.size());
    Put<std::uint32_t>(b,128,1);Put<std::uint32_t>(b,132,static_cast<std::uint32_t>(f.rgb_vertices.size()));Put<std::uint32_t>(b,136,static_cast<std::uint32_t>(f.indices.size()));
    Put(b,140,f.atlas_width);Put(b,144,f.atlas_height);Put<std::uint32_t>(b,152,static_cast<std::uint32_t>(b.size()-224));Put<std::uint32_t>(b,200,80);
    for(std::size_t i=0;i<n;++i){const auto& o=f.observations[i];const auto at=224+i*80;
        Put(b,at,o.camera_id);Put(b,at+4,o.flags);Put(b,at+8,o.observation_id);Put(b,at+16,o.depth_capture_ns);Put(b,at+24,o.image_capture_ns);
        const std::array<std::uint32_t,9> values{o.vertex_start,o.vertex_count,o.index_start,o.index_count,o.tile_x,o.tile_y,o.tile_width,o.tile_height,o.image_format};
        for(std::size_t j=0;j<values.size();++j)Put(b,at+32+j*4,values[j]);
    }
    const auto at=224+n*80;if(!f.rgb_vertices.empty())std::memcpy(b.data()+at,f.rgb_vertices.data(),f.rgb_vertices.size()*44);
    if(!f.indices.empty())std::memcpy(b.data()+at+f.rgb_vertices.size()*44,f.indices.data(),f.indices.size()*2);
    if(!f.atlas.empty())std::memcpy(b.data()+at+f.rgb_vertices.size()*44+f.indices.size()*2,f.atlas.data(),f.atlas.size());Crc(b);return b;
}
void Reject(const Bytes& b){RemoteSceneFrame f;f.sequence=99;f.rgb_vertices.push_back({});std::string e;
    Check(!DecodeRemoteScene(b,f,e)&&!e.empty()&&f.sequence==99&&f.rgb_vertices.size()==1,"invalid RGB packet accepted or changed output");}
Bytes Read(const char* path){std::ifstream s(path,std::ios::binary|std::ios::ate);Check(static_cast<bool>(s),"cannot open RGB fixture");const auto size=s.tellg();
    Check(size>=0&&size<=static_cast<std::streamoff>(kRemoteSceneMaxPacketBytes),"RGB fixture size exceeds bound");Bytes b(static_cast<std::size_t>(size));s.seekg(0);
    s.read(reinterpret_cast<char*>(b.data()),static_cast<std::streamsize>(b.size()));Check(static_cast<bool>(s),"RGB fixture read failed");return b;}
void Equal(const RemoteSceneFrame& a,const RemoteSceneFrame& b){
    Check(a.rgb_vertices.size()==b.rgb_vertices.size()&&a.indices==b.indices&&a.atlas==b.atlas&&a.observations==b.observations,
        "prepared/raw RGB counts, indices, atlas or observations differ");
    for(std::size_t i=0;i<a.rgb_vertices.size();++i){const auto& x=a.rgb_vertices[i];const auto& y=b.rgb_vertices[i];
        for(std::size_t j=0;j<3;++j)Check(std::abs(x.position[j]-y.position[j])<=1e-6F&&std::abs(x.uvq[j]-y.uvq[j])<=1e-6F,"prepared/raw RGB XYZ/UVQ differ");
        Check(x.tile_bounds==y.tile_bounds&&x.texture_valid==y.texture_valid,"prepared/raw RGB tile or visibility differ");
    }
}
}
int main(int argc,char** argv){try{RemoteSceneFrame f;std::string error;
    if(argc==3){RemoteSceneFrame b;
        if(!DecodeRemoteScene(Read(argv[1]),f,error))throw std::runtime_error("prepared: "+error);
        if(!DecodeRemoteScene(Read(argv[2]),b,error))throw std::runtime_error("raw: "+error);
        Equal(f,b);std::cout<<"RGB fixtures match: "<<f.rgb_vertices.size()<<" vertices, "<<f.indices.size()/3<<" triangles, "<<f.observations.size()<<" observations\n";return 0;
    }
    Check(argc==1,"expected zero arguments or prepared and raw fixtures");const auto raw=Raw();const auto good=DecodeRemoteScene(raw,f,error);
    if(!good)std::cerr<<error<<'\n';Check(good,"RGB v3 raw packet must decode");
    Check(f.wire_version==3&&f.atlas_channels==3&&f.rgb_vertices.size()==9&&f.indices.size()==24&&f.observations.size()==1&&f.atlas.size()==27,"raw RGB geometry or atlas missing");
    Check(f.atlas[0]==255&&f.atlas[1]==0&&f.atlas[2]==0&&f.rgb_vertices[0].texture_valid==1,"real RGB or image visibility changed");
    const auto prepared=Prepared(raw,f);RemoteSceneFrame b;Check(DecodeRemoteScene(prepared,b,error),error.c_str());Equal(f,b);
    auto unused=f;unused.indices.clear();unused.observations[0].index_count=0;
    Check(DecodeRemoteScene(Prepared(raw,unused),b,error)&&!b.HasGeometry(),"partitioned unused vertices are legal prepared metadata");
    auto outside=raw;Put(outside,224+40,100.F);Crc(outside);
    Check(DecodeRemoteScene(outside,b,error)&&b.rgb_vertices.size()==9&&b.rgb_vertices[0].texture_valid==1,"RGB FoV must not crop depth or corrupt projective interpolation");
    auto behind=raw;Put(behind,224+144,-2.F);Crc(behind);
    Check(DecodeRemoteScene(behind,b,error)&&b.rgb_vertices.size()==9&&b.rgb_vertices[0].uvq[2]<0,"behind-image depth geometry and signed Q must survive");
    auto unusable=raw;Put<std::uint32_t>(unusable,224+176,0);Crc(unusable);
    Check(DecodeRemoteScene(unusable,b,error)&&b.rgb_vertices.size()==9,"unusable image deleted depth geometry");
    for(const auto& v:b.rgb_vertices)Check(v.texture_valid==0,"unusable image labeled textured");
    auto stalled=unusable;
    for(std::size_t at:{40U,48U,56U,168U,224U+152})Put<std::uint64_t>(stalled,at,4000000000ULL);
    Put<std::uint64_t>(stalled,224+160,1);Crc(stalled);
    Check(DecodeRemoteScene(stalled,b,error)&&b.HasGeometry()&&b.observations[0].image_capture_ns==1&&b.oldest_observation_ns==4000000000ULL,
          "stalled unusable RGB must preserve fresh depth and original image provenance");
    Check(DecodeRemoteScene(Prepared(stalled,b),f,error),"prepared stalled-RGB geometry rejected");Equal(f,b);
    auto occluded=raw;Put(occluded,224+64,100.F);Put(occluded,224+32,.1F);Put(occluded,224+36,.1F);
    for(std::size_t y=0;y<3;++y)for(std::size_t x=0;x<2;++x)Put<std::uint16_t>(occluded,432+(y*3+x)*2,1020);Crc(occluded);
    Check(DecodeRemoteScene(occluded,b,error)&&b.rgb_vertices.size()==6,"occluded RGB must keep bounded depth geometry");
    for(const auto& v:b.rgb_vertices)Check(v.texture_valid==0,"out-of-map foreground must still occlude RGB correspondence");
    auto masked=raw;Put<std::uint8_t>(masked,masked.size()-1,0);Crc(masked);
    Check(DecodeRemoteScene(masked,b,error)&&b.rgb_vertices.empty()&&b.observations.size()==1&&b.atlas.size()==27,"triangle masks must suppress geometry without inventing missing observations");
    Check(DecodeRemoteScene(Prepared(masked,b),f,error),"prepared empty geometry must keep image tiles and metadata");
    auto retained=raw;Put<std::uint32_t>(retained,72,0);Put<std::uint32_t>(retained,76,kRemoteScenePartial);
    Put<std::uint64_t>(retained,40,0);Put<std::uint64_t>(retained,48,0);Put<std::uint64_t>(retained,56,2000000000);
    Put<std::uint32_t>(retained,176,0);Put<std::uint32_t>(retained,180,1);Put<std::uint32_t>(retained,188,3);Put<std::uint32_t>(retained,224+176,3);Crc(retained);
    Check(DecodeRemoteScene(retained,b,error)&&b.HasGeometry()&&b.contributing_mask==0&&b.retained_view_count==1,"retained-only observations must not claim current camera coverage");
    auto bad=retained;Put<std::uint32_t>(bad,184,999);Reject(bad);bad=retained;Put<std::uint32_t>(bad,188,1);Reject(bad);
    auto empty=Bytes(raw.begin(),raw.begin()+224);Put<std::uint32_t>(empty,72,0);Put<std::uint32_t>(empty,76,kRemoteScenePartial);
    Put<std::uint64_t>(empty,40,0);Put<std::uint64_t>(empty,48,0);Put<std::uint64_t>(empty,168,0);Put<std::uint64_t>(empty,160,2);
    Put<std::uint32_t>(empty,148,0);Put<std::uint32_t>(empty,152,0);Put<std::uint32_t>(empty,176,0);Put<std::uint32_t>(empty,188,0);Crc(empty);
    Check(DecodeRemoteScene(empty,b,error)&&!b.HasGeometry()&&b.observations.empty()&&b.map_generation==2,"tracking loss must retire geometry and advance map identity");
    Bytes ten(raw.begin(),raw.begin()+224);for(std::size_t i=0;i<10;++i){const auto at=ten.size();ten.insert(ten.end(),raw.begin()+224,raw.end());
        Put<std::uint32_t>(ten,at,static_cast<std::uint32_t>(i<6?i:i-6));Put<std::uint32_t>(ten,at+176,i<6?2U:3U);Put<std::uint64_t>(ten,at+168,i<6?1ULL:2ULL);}
    for(std::size_t at:{68U,72U})Put<std::uint32_t>(ten,at,63);Put<std::uint32_t>(ten,148,10);Put<std::uint32_t>(ten,176,6);
    Put<std::uint32_t>(ten,180,4);Put<std::uint32_t>(ten,188,3);Put<std::uint32_t>(ten,152,static_cast<std::uint32_t>(ten.size()-224));Crc(ten);
    Check(DecodeRemoteScene(ten,b,error)&&b.observations.size()==10&&b.rgb_vertices.size()==90&&b.atlas_width==15&&b.atlas_height==6,"six current plus four retained views must obey shelf layout");
    const auto ten_prepared=Prepared(ten,b);Check(DecodeRemoteScene(ten_prepared,f,error),"ten-view prepared packet rejected");Equal(f,b);
    for(std::size_t size=0;size<raw.size();++size)Reject(Bytes(raw.begin(),raw.begin()+size));
    for(std::size_t at:{128U,132U,136U,140U,144U,148U,152U,156U,176U,180U,184U,188U,192U,196U,200U,204U,208U,216U}){bad=raw;Put<std::uint32_t>(bad,at,0xffffffffU);Crc(bad);Reject(bad);}
    for(std::size_t at:{8U,16U,24U,32U,56U,160U,168U,224U+152,224U+160,224U+168}){bad=raw;Put<std::uint64_t>(bad,at,0);Crc(bad);Reject(bad);}
    for(std::size_t at:{224U+12,224U+148,224U+176,224U+180,224U+184,224U+188,224U+192,224U+200}){bad=raw;Put<std::uint32_t>(bad,at,0xffffffffU);Crc(bad);Reject(bad);}
    bad=raw;Put<std::uint32_t>(bad,224,6);Crc(bad);Reject(bad);bad=raw;Put<std::uint32_t>(bad,188,0);Reject(bad);
    bad=raw;Put(bad,224+16,0.F);Crc(bad);Reject(bad);bad=raw;Put(bad,224+52,-1.F);Crc(bad);Reject(bad);
    for(std::size_t at:{224U+16,224U+48,224U+52,224U+100}){bad=raw;Put(bad,at,std::numeric_limits<float>::quiet_NaN());Crc(bad);Reject(bad);}
    bad=raw;bad.push_back(std::byte{});Put<std::uint32_t>(bad,152,static_cast<std::uint32_t>(bad.size()-224));Crc(bad);Reject(bad);
    bad=raw;bad[440]^=std::byte{1};Reject(bad);bad=raw;Put(bad,80,-0.F);Reject(bad);
    const auto vertices_at=224+80;
    for(std::size_t at:{vertices_at,vertices_at+12,vertices_at+24,vertices_at+40}){bad=prepared;Put(bad,at,std::numeric_limits<float>::quiet_NaN());Crc(bad);Reject(bad);}
    bad=prepared;Put(bad,vertices_at+40,.5F);Crc(bad);Reject(bad);
    bad=prepared;Put(bad,vertices_at+24,.1F);Crc(bad);Reject(bad);
    bad=prepared;Put<std::uint32_t>(bad,224+4,0);Crc(bad);Reject(bad);
    for(std::size_t at:{224U+32,224U+36,224U+40,224U+44,224U+48,224U+52,224U+56,224U+64,224U+68,224U+72}){bad=prepared;Put<std::uint32_t>(bad,at,0xffffffffU);Crc(bad);Reject(bad);}
    bad=ten_prepared;const auto index_at=224+10*80+90*44;Put<std::uint16_t>(bad,index_at,9);Crc(bad);Reject(bad);
    RemoteSceneMailbox mailbox;const auto generation=mailbox.Reset();const auto now=RemoteSceneTime{}+std::chrono::seconds(10);
    Check(mailbox.Publish(raw,now,generation,error),"first RGB observation publishes");const auto first=mailbox.Read();
    Check(mailbox.Publish(empty,now+std::chrono::milliseconds(1),generation,error),"map reset may restart sequence");
    Check(!mailbox.Publish(raw,now+std::chrono::milliseconds(2),generation,error),"map reset A-B-A must not replay old sequence");
    auto returned=raw;Put<std::uint64_t>(returned,24,2);
    Check(mailbox.Publish(returned,now+std::chrono::milliseconds(3),generation,error)&&mailbox.Read().identity_revision==first.identity_revision+2,
          "coalesced map-generation changes must invalidate alignment");
    std::cout<<"RGB native checks passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
