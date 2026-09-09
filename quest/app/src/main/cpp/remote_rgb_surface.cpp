#include "remote_rgb_surface.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>

namespace quest_newton {
namespace {
using Vec3=std::array<double,3>;
std::uint64_t U(std::span<const std::byte> bytes,std::size_t at,unsigned width){
    std::uint64_t value=0;
    for(unsigned i=0;i<width;++i)value|=static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[at+i]))<<(i*8);
    return value;
}
float F(std::span<const std::byte> bytes,std::size_t at){return std::bit_cast<float>(static_cast<std::uint32_t>(U(bytes,at,4)));}
double AreaSquared(const Vec3& a,const Vec3& b,const Vec3& c){
    Vec3 ab{},ac{},cross{};
    for(std::size_t i=0;i<3;++i){ab[i]=b[i]-a[i];ac[i]=c[i]-a[i];}
    cross={ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]};
    return cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2];
}
struct View {
    std::uint32_t id=0,dw=0,dh=0,iw=0,ih=0;
    std::array<double,4> depth_intrinsics{},intensity_intrinsics{};
    std::array<double,12> world_from_depth{},intensity_from_depth{};
    double units=0;
    std::size_t depth_at=0,intensity_at=0,mask_at=0;
};
bool Rigid(const std::array<double,12>& m){
    for(double value:m)if(!std::isfinite(value))return false;
    for(std::size_t i=0;i<3;++i){
        if(std::abs(m[i*4+3])>100)return false;
        for(std::size_t j=0;j<3;++j){double rows=0,columns=0;
            for(std::size_t k=0;k<3;++k){rows+=m[i*4+k]*m[j*4+k];columns+=m[k*4+i]*m[k*4+j];}
            if(std::abs(rows-(i==j?1.:0.))>1e-3||std::abs(columns-(i==j?1.:0.))>1e-3)return false;
        }
    }
    const auto det=m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])+m[2]*(m[4]*m[9]-m[5]*m[8]);
    return std::abs(det-1.)<=1e-3;
}
bool Intrinsics(const std::array<double,4>& k){
    return std::all_of(k.begin(),k.end(),[](double x){return std::isfinite(x);})&&k[0]>0&&k[1]>0;
}
Vec3 Transform(const std::array<double,12>& m,const Vec3& point){
    Vec3 result{};
    for(std::size_t i=0;i<3;++i)result[i]=m[i*4]*point[0]+m[i*4+1]*point[1]+m[i*4+2]*point[2]+m[i*4+3];
    return result;
}
std::array<float,4> Bounds(const RemoteSceneObservation& o,std::uint32_t w,std::uint32_t h){
    return {static_cast<float>((o.tile_x+.5)/w),static_cast<float>((o.tile_y+.5)/h),
        static_cast<float>((o.tile_x+o.tile_width-.5)/w),static_cast<float>((o.tile_y+o.tile_height-.5)/h)};
}
RemoteSceneRgbVertex Vertex(std::span<const std::byte> p,std::size_t at){
    RemoteSceneRgbVertex v;
    for(std::size_t i=0;i<3;++i){v.position[i]=F(p,at+i*4);v.uvq[i]=F(p,at+12+i*4);}
    for(std::size_t i=0;i<4;++i)v.tile_bounds[i]=F(p,at+24+i*4);
    v.texture_valid=F(p,at+40);return v;
}
bool ValidVertex(const RemoteSceneRgbVertex& v,const std::array<float,4>& bounds,bool usable){
    for(float x:v.position)if(!std::isfinite(x)||std::abs(x)>100)return false;
    for(float x:v.uvq)if(!std::isfinite(x))return false;
    return v.tile_bounds==bounds&&(v.texture_valid==0||(usable&&v.texture_valid==1));
}
struct Sample {
    Vec3 world{};RemoteSceneRgbVertex vertex;
    double depth=0,image_depth=0;std::size_t pixel=0;
    bool valid=false,projects=false;
};
void Cook(std::span<const std::byte> p,const View& v,RemoteSceneObservation& o,RemoteSceneFrame& f){
    const auto count=static_cast<std::size_t>(v.dw)*v.dh;
    std::vector<Sample> samples(count);std::vector<double> nearest(static_cast<std::size_t>(v.iw)*v.ih,std::numeric_limits<double>::infinity());
    const auto bounds=Bounds(o,f.atlas_width,f.atlas_height);const auto& dk=v.depth_intrinsics;const auto& ik=v.intensity_intrinsics;
    for(std::uint32_t y=0;y<v.dh;++y)for(std::uint32_t x=0;x<v.dw;++x){
        const auto i=static_cast<std::size_t>(y)*v.dw+x;auto& s=samples[i];s.depth=static_cast<double>(U(p,v.depth_at+i*2,2))*v.units;
        if(s.depth<=0)continue;
        const Vec3 dp={(static_cast<double>(x)-dk[2])*s.depth/dk[0],(static_cast<double>(y)-dk[3])*s.depth/dk[1],s.depth};
        s.world=Transform(v.world_from_depth,dp);const auto ip=Transform(v.intensity_from_depth,dp);
        s.vertex.tile_bounds=bounds;
        const std::array<double,3> uvq={(ik[0]*ip[0]+(ik[2]+.5+o.tile_x)*ip[2])/f.atlas_width,
            (ik[1]*ip[1]+(ik[3]+.5+o.tile_y)*ip[2])/f.atlas_height,ip[2]};
        const bool finite=std::all_of(uvq.begin(),uvq.end(),[](double z){return std::isfinite(z)&&std::abs(z)<=std::numeric_limits<float>::max();});
        if(finite)for(std::size_t k=0;k<3;++k)s.vertex.uvq[k]=static_cast<float>(uvq[k]);
        s.vertex.texture_valid=(finite&&(o.flags&2U))?1.F:0.F;
        if(finite&&ip[2]>0){
            const double u=ik[0]*ip[0]/ip[2]+ik[2],w=ik[1]*ip[1]/ip[2]+ik[3];
            if(std::isfinite(u)&&std::isfinite(w)&&u>=0&&u<=v.iw-1&&w>=0&&w<=v.ih-1){
                s.pixel=static_cast<std::size_t>(std::floor(w+.5))*v.iw+static_cast<std::size_t>(std::floor(u+.5));
                s.image_depth=ip[2];s.projects=true;nearest[s.pixel]=std::min(nearest[s.pixel],ip[2]);
            }
        }
        if(!std::all_of(s.world.begin(),s.world.end(),[](double z){return std::isfinite(z)&&std::abs(z)<=100.;}))continue;
        for(std::size_t k=0;k<3;++k)s.vertex.position[k]=static_cast<float>(s.world[k]);s.valid=true;
    }
    for(auto& s:samples)if(s.projects&&s.image_depth>nearest[s.pixel]+.005)s.vertex.texture_valid=0;
    std::vector<std::array<std::uint16_t,3>> triangles;std::vector<bool> used(count,false);
    const auto triangle=[&](std::size_t a,std::size_t b,std::size_t c,std::size_t number){
        if(!(U(p,v.mask_at+number/8,1)&(1ULL<<(number%8))))return;
        if(!samples[a].valid||!samples[b].valid||!samples[c].valid)return;
        const std::array<std::size_t,3> ids{a,b,c};std::array<Vec3,3> emitted{};
        for(std::size_t j=0;j<3;++j)for(std::size_t k=0;k<3;++k)emitted[j][k]=samples[ids[j]].vertex.position[k];
        if(AreaSquared(samples[a].world,samples[b].world,samples[c].world)<=1e-12||AreaSquared(emitted[0],emitted[1],emitted[2])<=1e-12)return;
        for(std::size_t j=0;j<3;++j){const auto& x=samples[ids[j]];const auto& y=samples[ids[(j+1)%3]];double len=0;
            for(std::size_t k=0;k<3;++k){const auto d=x.world[k]-y.world[k];len+=d*d;}
            if(len>.25||std::abs(x.depth-y.depth)>.05+.02*std::min(x.depth,y.depth))return;
        }
        triangles.push_back({static_cast<std::uint16_t>(a),static_cast<std::uint16_t>(b),static_cast<std::uint16_t>(c)});used[a]=used[b]=used[c]=true;
    };
    std::size_t number=0;
    for(std::uint32_t y=0;y+1<v.dh;++y)for(std::uint32_t x=0;x+1<v.dw;++x){
        const std::size_t a=static_cast<std::size_t>(y)*v.dw+x,b=a+1,c=a+v.dw,d=c+1;triangle(a,c,b,number++);triangle(b,c,d,number++);
    }
    o.vertex_start=static_cast<std::uint32_t>(f.rgb_vertices.size());o.index_start=static_cast<std::uint32_t>(f.indices.size());
    std::vector<std::uint16_t> remap(count);
    for(std::size_t i=0;i<count;++i)if(used[i]){remap[i]=static_cast<std::uint16_t>(f.rgb_vertices.size());f.rgb_vertices.push_back(samples[i].vertex);}
    for(const auto& t:triangles)for(auto i:t)f.indices.push_back(remap[i]);
    o.vertex_count=static_cast<std::uint32_t>(f.rgb_vertices.size())-o.vertex_start;o.index_count=static_cast<std::uint32_t>(f.indices.size())-o.index_start;
}
bool Metadata(const std::array<RemoteSceneObservation,10>& observations,std::size_t count,const RemoteSceneFrame& f){
    std::uint32_t mask=0;std::uint64_t oldest=std::numeric_limits<std::uint64_t>::max();
    for(std::size_t i=0;i<count;++i){const auto& o=observations[i];const bool retained=(o.flags&1U)!=0;
        if(o.camera_id>5||(o.flags&~3U)||!o.observation_id||!o.depth_capture_ns||!o.image_capture_ns||
           o.depth_capture_ns>f.produced_ns||o.image_capture_ns>f.produced_ns||retained!=(i>=f.current_view_count)||
           (o.image_format!=1&&o.image_format!=3)||o.tile_width<2||o.tile_width>256||o.tile_height<2||o.tile_height>192)return false;
        const auto stamp=RemoteObservationCaptureNs(o);oldest=std::min(oldest,stamp);
        if(retained){
            if(!(f.map_flags&2U)||f.produced_ns-stamp>static_cast<std::uint64_t>(f.retention_ms)*1000000ULL)return false;
            if(i>f.current_view_count){const auto& last=observations[i-1];
                if(o.camera_id<last.camera_id||(o.camera_id==last.camera_id&&o.observation_id<=last.observation_id))return false;
            }
        }else{
            if((i&&o.camera_id<=observations[i-1].camera_id)||o.depth_capture_ns<f.capture_start_ns||o.depth_capture_ns>f.capture_end_ns||
               ((o.flags&2U)&&f.produced_ns-o.image_capture_ns>2000000000ULL))return false;
            mask|=1U<<o.camera_id;
        }
        for(std::size_t j=0;j<i;++j)if(observations[j].camera_id==o.camera_id&&observations[j].observation_id==o.observation_id)return false;
    }
    return mask==f.contributing_mask&&(count?oldest:0)==f.oldest_observation_ns;
}
void Layout(std::array<RemoteSceneObservation,10>& observations,std::size_t count,std::uint32_t& width,std::uint32_t& height){
    width=height=0;
    for(std::size_t start=0;start<count;start+=5){std::uint32_t row_width=0,row_height=0;
        for(std::size_t i=start;i<std::min(start+5,count);++i){auto& o=observations[i];o.tile_x=row_width;o.tile_y=height;row_width+=o.tile_width;row_height=std::max(row_height,o.tile_height);}
        width=std::max(width,row_width);height+=row_height;
    }
}
}
bool DecodeRemoteRgbSurface(std::span<const std::byte> p,RemoteSceneFrame& f,std::string& error){
    const auto reject=[&](const char* message){error=message;return false;};
    if(p.size()<224)return reject("RGB header truncated");
    const auto representation=U(p,128,4),vc=U(p,132,4),ic=U(p,136,4),aw=U(p,140,4),ah=U(p,144,4),count=U(p,148,4);
    f.map_generation=U(p,160,8);f.oldest_observation_ns=U(p,168,8);
    f.current_view_count=static_cast<std::uint32_t>(U(p,176,4));f.retained_view_count=static_cast<std::uint32_t>(U(p,180,4));
    f.retention_ms=static_cast<std::uint32_t>(U(p,184,4));f.map_flags=static_cast<std::uint32_t>(U(p,188,4));
    if((representation!=1&&representation!=2)||count>10||!f.map_generation||f.current_view_count>6||f.retained_view_count>4||
       count!=static_cast<std::uint64_t>(f.current_view_count)+f.retained_view_count||f.current_view_count!=static_cast<unsigned>(std::popcount(f.contributing_mask))||
       f.retention_ms<1||f.retention_ms>60000||(f.map_flags&~3U)||(!(f.map_flags&1U)&&(count||vc||ic||aw||ah))||
       U(p,152,4)!=p.size()-224||U(p,156,4)||U(p,192,4)!=44||U(p,196,4)!=3||U(p,200,4)!=(representation==1?80U:208U)||
       U(p,204,4)||U(p,208,8)||U(p,216,8))return reject("Invalid RGB header or observation counts");
    f.representation=static_cast<RemoteSceneRepresentation>(representation);f.atlas_channels=3;
    std::array<RemoteSceneObservation,10> observations{};std::array<View,10> views{};
    if(representation==1){
        if(vc>kRemoteRgbMaxVertices||ic>kRemoteRgbMaxIndices||ic%3||aw>2048||ah>2048||aw*ah>kRemoteRgbMaxPixels||
           count*80+vc*44+ic*2+aw*ah*3!=p.size()-224)return reject("Invalid prepared RGB lengths or budgets");
        std::uint64_t vertex_end=0,index_end=0;
        for(std::size_t i=0;i<count;++i){auto& o=observations[i];const auto at=224+i*80;
            o.camera_id=static_cast<std::uint32_t>(U(p,at,4));o.flags=static_cast<std::uint32_t>(U(p,at+4,4));o.observation_id=U(p,at+8,8);
            o.depth_capture_ns=U(p,at+16,8);o.image_capture_ns=U(p,at+24,8);
            o.vertex_start=static_cast<std::uint32_t>(U(p,at+32,4));o.vertex_count=static_cast<std::uint32_t>(U(p,at+36,4));
            o.index_start=static_cast<std::uint32_t>(U(p,at+40,4));o.index_count=static_cast<std::uint32_t>(U(p,at+44,4));
            o.tile_x=static_cast<std::uint32_t>(U(p,at+48,4));o.tile_y=static_cast<std::uint32_t>(U(p,at+52,4));
            o.tile_width=static_cast<std::uint32_t>(U(p,at+56,4));o.tile_height=static_cast<std::uint32_t>(U(p,at+60,4));o.image_format=static_cast<std::uint32_t>(U(p,at+64,4));
            if(U(p,at+68,4)||U(p,at+72,8)||o.vertex_start!=vertex_end||o.index_start!=index_end||o.index_count%3)return reject("Invalid RGB observation ranges");
            vertex_end+=o.vertex_count;index_end+=o.index_count;
            if(vertex_end>vc||index_end>ic)return reject("RGB observation range outside arrays");
        }
        if(vertex_end!=vc||index_end!=ic)return reject("RGB observation ranges do not partition arrays");
        if(!Metadata(observations,static_cast<std::size_t>(count),f))return reject("Invalid RGB observation identity, timing or format");
        auto expected=observations;std::uint32_t width=0,height=0;Layout(expected,static_cast<std::size_t>(count),width,height);
        if(aw!=width||ah!=height)return reject("RGB atlas dimensions do not match observation layout");
        const auto vertices_at=224+static_cast<std::size_t>(count)*80,index_at=vertices_at+static_cast<std::size_t>(vc)*44;
        for(std::size_t i=0;i<count;++i){const auto& o=observations[i];
            if(o.tile_x!=expected[i].tile_x||o.tile_y!=expected[i].tile_y)return reject("RGB observation tile layout mismatch");
            const auto bounds=Bounds(o,width,height);
            for(std::size_t j=o.vertex_start;j<static_cast<std::size_t>(o.vertex_start)+o.vertex_count;++j)
                if(!ValidVertex(Vertex(p,vertices_at+j*44),bounds,(o.flags&2U)!=0))return reject("Invalid RGB vertex or observation tile bounds");
            for(std::size_t j=o.index_start;j<static_cast<std::size_t>(o.index_start)+o.index_count;j+=3){std::array<Vec3,3> triangle{};std::array<std::uint64_t,3> indices{};
                for(std::size_t k=0;k<3;++k){const auto index=U(p,index_at+(j+k)*2,2);indices[k]=index;
                    if(index<o.vertex_start||index>=static_cast<std::uint64_t>(o.vertex_start)+o.vertex_count)return reject("RGB triangle crosses observation range");
                    const auto v=Vertex(p,vertices_at+static_cast<std::size_t>(index)*44);for(std::size_t axis=0;axis<3;++axis)triangle[k][axis]=v.position[axis];
                }
                if(indices[0]==indices[1]||indices[0]==indices[2]||indices[1]==indices[2]||AreaSquared(triangle[0],triangle[1],triangle[2])<=1e-12)return reject("Degenerate RGB triangle");
            }
        }
        f.rgb_vertices.reserve(static_cast<std::size_t>(vc));f.indices.reserve(static_cast<std::size_t>(ic));
        for(std::size_t i=0;i<vc;++i)f.rgb_vertices.push_back(Vertex(p,vertices_at+i*44));
        for(std::size_t i=0;i<ic;++i)f.indices.push_back(static_cast<std::uint16_t>(U(p,index_at+i*2,2)));
        f.atlas_width=width;f.atlas_height=height;
        f.atlas.reserve(static_cast<std::size_t>(aw*ah*3));
        for(std::size_t i=index_at+static_cast<std::size_t>(ic)*2;i<p.size();++i)f.atlas.push_back(std::to_integer<std::uint8_t>(p[i]));
    }else{
        if(vc||ic||aw||ah)return reject("Raw RGB packet declares prepared arrays");
        std::size_t offset=224;
        for(std::size_t i=0;i<count;++i){
            if(p.size()-offset<208)return reject("Truncated RGB descriptor");auto& v=views[i];auto& o=observations[i];
            o.camera_id=v.id=static_cast<std::uint32_t>(U(p,offset,4));v.dw=static_cast<std::uint32_t>(U(p,offset+4,2));v.dh=static_cast<std::uint32_t>(U(p,offset+6,2));
            o.tile_width=v.iw=static_cast<std::uint32_t>(U(p,offset+8,2));o.tile_height=v.ih=static_cast<std::uint32_t>(U(p,offset+10,2));
            o.depth_capture_ns=U(p,offset+152,8);o.image_capture_ns=U(p,offset+160,8);o.observation_id=U(p,offset+168,8);
            o.flags=static_cast<std::uint32_t>(U(p,offset+176,4));o.image_format=static_cast<std::uint32_t>(U(p,offset+180,4));
            if(v.dw<2||v.dw>64||v.dh<2||v.dh>48||v.iw<2||v.iw>256||v.ih<2||v.ih>192||(o.image_format!=1&&o.image_format!=3)||
               U(p,offset+12,4)||U(p,offset+148,4)||U(p,offset+188,4)||U(p,offset+192,8)||U(p,offset+200,8))return reject("Invalid RGB image dimensions or descriptor reserved fields");
            const std::size_t triangle_count=2*static_cast<std::size_t>(v.dw-1)*(v.dh-1),mask_bytes=(triangle_count+7)/8;
            if(U(p,offset+184,4)!=mask_bytes)return reject("RGB triangle-mask length mismatch");
            for(std::size_t k=0;k<4;++k){v.depth_intrinsics[k]=F(p,offset+16+k*4);v.intensity_intrinsics[k]=F(p,offset+32+k*4);}
            v.units=F(p,offset+48);
            for(std::size_t k=0;k<12;++k){v.world_from_depth[k]=F(p,offset+52+k*4);v.intensity_from_depth[k]=F(p,offset+100+k*4);}
            if(!Intrinsics(v.depth_intrinsics)||!Intrinsics(v.intensity_intrinsics)||!std::isfinite(v.units)||v.units<=0||v.units>1||!Rigid(v.world_from_depth)||!Rigid(v.intensity_from_depth))return reject("Invalid RGB calibration");
            v.depth_at=offset+208;v.intensity_at=v.depth_at+static_cast<std::size_t>(v.dw)*v.dh*2;
            v.mask_at=v.intensity_at+static_cast<std::size_t>(v.iw)*v.ih*o.image_format;
            if(v.mask_at+mask_bytes>p.size())return reject("Truncated RGB images or mask");
            if(triangle_count%8&&(U(p,v.mask_at+mask_bytes-1,1)&~((1ULL<<(triangle_count%8))-1ULL)))return reject("RGB triangle mask unused bits must be zero");
            offset=v.mask_at+mask_bytes;
        }
        if(offset!=p.size()||!Metadata(observations,static_cast<std::size_t>(count),f))return reject("Invalid RGB observation metadata or payload length");
        Layout(observations,static_cast<std::size_t>(count),f.atlas_width,f.atlas_height);
        f.atlas.resize(static_cast<std::size_t>(f.atlas_width)*f.atlas_height*3,0);
        for(std::size_t i=0;i<count;++i){auto& o=observations[i];const auto& v=views[i];
            for(std::uint32_t y=0;y<v.ih;++y)for(std::uint32_t x=0;x<v.iw;++x)for(std::size_t c=0;c<3;++c){
                const auto source=v.intensity_at+(static_cast<std::size_t>(y)*v.iw+x)*o.image_format+(o.image_format==3?c:0);
                f.atlas[((static_cast<std::size_t>(y)+o.tile_y)*f.atlas_width+o.tile_x+x)*3+c]=std::to_integer<std::uint8_t>(p[source]);
            }
            Cook(p,v,o,f);
        }
    }
    f.observations.assign(observations.begin(),observations.begin()+static_cast<std::ptrdiff_t>(count));return true;
}
} // namespace quest_newton
