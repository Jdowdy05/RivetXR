#include "remote_surface.h"
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
Vec3 Position(std::span<const std::byte> packet,std::size_t index){
    const auto at=kRemoteSurfaceHeaderBytes+index*kRemoteSurfaceVertexBytes;
    return {F(packet,at),F(packet,at+4),F(packet,at+8)};
}
bool ValidVertex(const RemoteSceneVertex& vertex){
    for(float value:vertex.position)if(!std::isfinite(value)||std::abs(value)>100.F)return false;
    for(float value:vertex.uvq)if(!std::isfinite(value))return false;
    const double q=vertex.uvq[2];
    return q>=1e-6F&&q<=1024&&vertex.uvq[0]>=0&&vertex.uvq[0]<=q&&vertex.uvq[1]>=0&&vertex.uvq[1]<=q;
}
RemoteSceneVertex Vertex(std::span<const std::byte> packet,std::size_t index){
    RemoteSceneVertex vertex;const auto at=kRemoteSurfaceHeaderBytes+index*kRemoteSurfaceVertexBytes;
    for(std::size_t axis=0;axis<3;++axis){vertex.position[axis]=F(packet,at+axis*4);vertex.uvq[axis]=F(packet,at+12+axis*4);}
    return vertex;
}
struct View {
    std::uint32_t id=0,dw=0,dh=0,iw=0,ih=0;
    std::array<double,4> depth_intrinsics{},intensity_intrinsics{};
    std::array<double,12> world_from_depth{},intensity_from_depth{};
    double units=0;
    std::size_t depth_at=0,intensity_at=0;
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
struct Sample {
    Vec3 world{};
    RemoteSceneVertex vertex;
    double depth=0,intensity_depth=0;
    std::size_t image_pixel=0;
    bool valid=false;
};
void BuildView(std::span<const std::byte> packet,const View& view,std::uint32_t tile,RemoteSceneFrame& frame){
    const auto count=static_cast<std::size_t>(view.dw)*view.dh;
    std::vector<Sample> samples(count);
    std::vector<double> nearest(static_cast<std::size_t>(view.iw)*view.ih,std::numeric_limits<double>::infinity());
    const auto& dk=view.depth_intrinsics;const auto& ik=view.intensity_intrinsics;
    // All arithmetic uses float64 over canonical float32 wire calibration.
    for(std::uint32_t y=0;y<view.dh;++y)for(std::uint32_t x=0;x<view.dw;++x){
        const auto i=static_cast<std::size_t>(y)*view.dw+x;auto& s=samples[i];
        s.depth=static_cast<double>(U(packet,view.depth_at+i*2,2))*view.units;
        if(s.depth<=0)continue;
        const Vec3 dp={(static_cast<double>(x)-dk[2])*s.depth/dk[0],(static_cast<double>(y)-dk[3])*s.depth/dk[1],s.depth};
        s.world=Transform(view.world_from_depth,dp);const auto ip=Transform(view.intensity_from_depth,dp);
        if(!std::all_of(ip.begin(),ip.end(),[](double v){return std::isfinite(v);})||ip[2]<1e-6||ip[2]>1024.)continue;
        const double u=ik[0]*ip[0]/ip[2]+ik[2],v=ik[1]*ip[1]/ip[2]+ik[3];
        if(!std::isfinite(u)||!std::isfinite(v)||u<0||u>view.iw-1||v<0||v>view.ih-1)continue;
        const auto ix=static_cast<std::uint32_t>(std::floor(u+.5)),iy=static_cast<std::uint32_t>(std::floor(v+.5));
        s.image_pixel=static_cast<std::size_t>(iy)*view.iw+ix;s.intensity_depth=ip[2];
        // A foreground sample can occlude this image pixel even when its world
        // position is clipped from the rendered map. Match producer visibility.
        nearest[s.image_pixel]=std::min(nearest[s.image_pixel],ip[2]);
        if(!std::all_of(s.world.begin(),s.world.end(),[](double v){return std::isfinite(v)&&std::abs(v)<=100.;}))continue;
        for(std::size_t axis=0;axis<3;++axis)s.vertex.position[axis]=static_cast<float>(s.world[axis]);
        s.vertex.uvq={static_cast<float>((ik[0]*ip[0]+(ik[2]+.5+tile)*ip[2])/frame.atlas_width),
                      static_cast<float>((ik[1]*ip[1]+(ik[3]+.5)*ip[2])/frame.atlas_height),static_cast<float>(ip[2])};
        if(!ValidVertex(s.vertex))continue;
        s.valid=true;
    }
    for(auto& s:samples)if(s.valid&&s.intensity_depth>nearest[s.image_pixel]+.005)s.valid=false;
    std::vector<std::array<std::uint16_t,3>> triangles;
    triangles.reserve(static_cast<std::size_t>(view.dw-1)*(view.dh-1)*2);
    std::vector<bool> used(count,false);
    const auto triangle=[&](std::size_t a,std::size_t b,std::size_t c){
        const std::array<std::size_t,3> ids={a,b,c};
        if(!samples[a].valid||!samples[b].valid||!samples[c].valid||AreaSquared(samples[a].world,samples[b].world,samples[c].world)<=1e-12)return;
        std::array<Vec3,3> emitted{};
        for(std::size_t i=0;i<3;++i)for(std::size_t axis=0;axis<3;++axis)emitted[i][axis]=samples[ids[i]].vertex.position[axis];
        if(AreaSquared(emitted[0],emitted[1],emitted[2])<=1e-12)return;
        for(std::size_t e=0;e<3;++e){const auto& p=samples[ids[e]];const auto& q=samples[ids[(e+1)%3]];double length2=0;
            for(std::size_t axis=0;axis<3;++axis){const auto d=p.world[axis]-q.world[axis];length2+=d*d;}
            if(length2>.25||std::abs(p.depth-q.depth)>.05+.02*std::min(p.depth,q.depth))return;
        }
        triangles.push_back({static_cast<std::uint16_t>(a),static_cast<std::uint16_t>(b),static_cast<std::uint16_t>(c)});
        used[a]=used[b]=used[c]=true;
    };
    for(std::uint32_t y=0;y+1<view.dh;++y)for(std::uint32_t x=0;x+1<view.dw;++x){
        const std::size_t a=static_cast<std::size_t>(y)*view.dw+x,b=a+1,c=a+view.dw,d=c+1;
        triangle(a,c,b);triangle(b,c,d);
    }
    std::vector<std::uint16_t> remap(count);
    for(std::size_t i=0;i<count;++i)if(used[i]){remap[i]=static_cast<std::uint16_t>(frame.vertices.size());frame.vertices.push_back(samples[i].vertex);}
    for(const auto& t:triangles)for(const auto i:t)frame.indices.push_back(remap[i]);
}
}
bool DecodeRemoteSurface(std::span<const std::byte> packet,RemoteSceneFrame& frame,std::string& error){
    const auto reject=[&](const char* message){error=message;return false;};
    // This function is internal, but still rejects short input independently.
    if(packet.size()<kRemoteSurfaceHeaderBytes)return reject("RSCN surface header truncated");
    const auto representation=U(packet,128,4),vc=U(packet,132,4),ic=U(packet,136,4),aw=U(packet,140,4),ah=U(packet,144,4),views=U(packet,148,4);
    if((representation!=1&&representation!=2)||U(packet,156,4)||views!=static_cast<unsigned>(std::popcount(frame.contributing_mask))||
       U(packet,152,4)!=packet.size()-kRemoteSurfaceHeaderBytes)return reject("Invalid RSCN surface extension or payload length");
    frame.representation=static_cast<RemoteSceneRepresentation>(representation);
    if(representation==1){
        if(vc>kRemoteSurfaceMaxVertices||ic>kRemoteSurfaceMaxIndices||ic%3||aw>kRemoteSurfaceMaxAtlasDimension||ah>kRemoteSurfaceMaxAtlasDimension||
           aw*ah>kRemoteSurfaceMaxAtlasPixels||vc*kRemoteSurfaceVertexBytes+ic*2+aw*ah!=packet.size()-kRemoteSurfaceHeaderBytes)
            return reject("RSCN prepared surface exceeds budgets or exact length");
        const bool empty=vc==0;
        if(empty!=(ic==0)||empty!=(aw==0)||empty!=(ah==0)||(!frame.contributing_mask&&!empty))return reject("Inconsistent RSCN surface emptiness");
        for(std::size_t i=0;i<vc;++i)if(!ValidVertex(Vertex(packet,i)))return reject("Invalid RSCN surface XYZ or projective texture coordinates");
        const auto index_at=kRemoteSurfaceHeaderBytes+static_cast<std::size_t>(vc)*kRemoteSurfaceVertexBytes;
        for(std::size_t i=0;i<ic;i+=3){
            const auto a=U(packet,index_at+i*2,2),b=U(packet,index_at+(i+1)*2,2),c=U(packet,index_at+(i+2)*2,2);
            if(a>=vc||b>=vc||c>=vc||a==b||a==c||b==c||AreaSquared(Position(packet,static_cast<std::size_t>(a)),Position(packet,static_cast<std::size_t>(b)),Position(packet,static_cast<std::size_t>(c)))<=1e-12)
                return reject("Invalid RSCN surface index or degenerate triangle");
        }
        frame.vertices.reserve(static_cast<std::size_t>(vc));frame.indices.reserve(static_cast<std::size_t>(ic));
        for(std::size_t i=0;i<vc;++i)frame.vertices.push_back(Vertex(packet,i));
        for(std::size_t i=0;i<ic;++i)frame.indices.push_back(static_cast<std::uint16_t>(U(packet,index_at+i*2,2)));
        const auto atlas_at=index_at+static_cast<std::size_t>(ic)*2;
        frame.atlas.reserve(static_cast<std::size_t>(aw*ah));
        for(std::size_t i=atlas_at;i<packet.size();++i)frame.atlas.push_back(std::to_integer<std::uint8_t>(packet[i]));
        frame.atlas_width=static_cast<std::uint32_t>(aw);frame.atlas_height=static_cast<std::uint32_t>(ah);return true;
    }
    if(vc||ic||aw||ah)return reject("Unprepared RSCN surface must not contain prepared counts");
    std::array<View,5> descriptors{};std::size_t offset=kRemoteSurfaceHeaderBytes;
    std::uint32_t mask=0,width=0,height=0;
    for(std::size_t i=0;i<views;++i){
        if(packet.size()-offset<kRemoteSurfaceViewBytes)return reject("Truncated RSCN surface camera descriptor");
        auto& v=descriptors[i];v.id=static_cast<std::uint32_t>(U(packet,offset,4));
        v.dw=static_cast<std::uint32_t>(U(packet,offset+4,2));v.dh=static_cast<std::uint32_t>(U(packet,offset+6,2));
        v.iw=static_cast<std::uint32_t>(U(packet,offset+8,2));v.ih=static_cast<std::uint32_t>(U(packet,offset+10,2));
        if(v.id>4||(i&&v.id<=descriptors[i-1].id)||v.dw<2||v.dw>64||v.dh<2||v.dh>48||v.iw<2||v.iw>320||v.ih<2||v.ih>240||
           U(packet,offset+12,4)||U(packet,offset+148,4))return reject("Invalid RSCN surface camera ID, image dimensions or reserved field");
        mask|=1U<<v.id;width+=v.iw;height=std::max(height,v.ih);
        for(std::size_t j=0;j<4;++j){v.depth_intrinsics[j]=F(packet,offset+16+j*4);v.intensity_intrinsics[j]=F(packet,offset+32+j*4);}
        v.units=F(packet,offset+48);
        for(std::size_t j=0;j<12;++j){v.world_from_depth[j]=F(packet,offset+52+j*4);v.intensity_from_depth[j]=F(packet,offset+100+j*4);}
        const auto capture=U(packet,offset+152,8);
        if(!Intrinsics(v.depth_intrinsics)||!Intrinsics(v.intensity_intrinsics)||!std::isfinite(v.units)||v.units<=0||v.units>1||
           !Rigid(v.world_from_depth)||!Rigid(v.intensity_from_depth)||capture<frame.capture_start_ns||capture>frame.capture_end_ns)
            return reject("Invalid RSCN surface calibration or exposure time");
        v.depth_at=offset+kRemoteSurfaceViewBytes;const auto depth_bytes=static_cast<std::size_t>(v.dw)*v.dh*2;
        const auto intensity_bytes=static_cast<std::size_t>(v.iw)*v.ih;
        if(depth_bytes+intensity_bytes>packet.size()-v.depth_at)return reject("Truncated RSCN surface camera images");
        v.intensity_at=v.depth_at+depth_bytes;offset=v.intensity_at+intensity_bytes;
    }
    if(offset!=packet.size()||mask!=frame.contributing_mask)return reject("RSCN surface camera IDs or exact payload length mismatch");
    frame.atlas_width=width;frame.atlas_height=height;frame.atlas.resize(static_cast<std::size_t>(width)*height,0);
    std::uint32_t tile=0;
    for(std::size_t i=0;i<views;++i){const auto& v=descriptors[i];
        for(std::uint32_t y=0;y<v.ih;++y)for(std::uint32_t x=0;x<v.iw;++x)
            frame.atlas[static_cast<std::size_t>(y)*width+tile+x]=std::to_integer<std::uint8_t>(packet[v.intensity_at+static_cast<std::size_t>(y)*v.iw+x]);
        BuildView(packet,v,tile,frame);tile+=v.iw;
    }
    if(frame.indices.empty()){frame.vertices.clear();frame.atlas.clear();frame.atlas_width=frame.atlas_height=0;}
    return true;
}
} // namespace quest_newton
