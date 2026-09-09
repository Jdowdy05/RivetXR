#include "remote_scene_renderer.h"
#include <iostream>
#include <stdexcept>
namespace quest_newton {Mat4 Mat4::Identity(){Mat4 m;for(int i=0;i<4;++i)m.m[i*5]=1;return m;}}
namespace {
using namespace quest_newton;
void Check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
RemoteSceneSnapshot Snapshot(std::uint64_t publication,bool mesh){
 auto frame=std::make_shared<RemoteSceneFrame>();
 if(mesh){frame->representation=RemoteSceneRepresentation::Prepared;
  frame->vertices={{{0,0,-1},{.25F,.25F,1}},{{1,0,-1},{.75F,.25F,1}},{{0,1,-1},{.25F,.75F,1}}};
  frame->indices={0,1,2};frame->atlas={1,2,3,4};frame->atlas_width=frame->atlas_height=2;
 }else frame->points={{{0,0,-1},128,1,5}};
 RemoteSceneSnapshot s;s.frame=frame;s.stream_generation=1;s.publication=publication;return s;
}
std::vector<OVRFW::ovrDrawSurface> Draw(RemoteSceneRenderer& r){std::vector<OVRFW::ovrDrawSurface> out;r.Append(Mat4::Identity(),2000,false,true,out);return out;}
}
int main(){try{
 const auto initial=mock_gl::State();
 unsigned after_shutdown=0;
 {
  RemoteSceneRenderer r;Check(r.Init(),"renderer init failed");Check(mock_gl::State()==initial,"Init leaked GL binding/unpack/program state");
  Check(r.Upload(Snapshot(1,false)),"legacy point upload failed");
  Check(Draw(r).size()==2,"legacy point geometry missing");
  Check(r.Upload(Snapshot(2,true)),"mesh upload failed");
  auto before=Draw(r);Check(before.size()==2,"textured geometry was not published");
  const auto* surface=before.back().surface;
  Check(surface->geo.primitiveType==GL_TRIANGLES&&surface->geo.indexCount==3,"mesh did not publish triangle indices");
  Check(surface->graphicsCommand.GpuState.cullEnable&&surface->graphicsCommand.GpuState.frontFace==GL_CCW,"mesh backface policy incorrect");
  const auto attrib=mock_gl::attributes.at({surface->geo.vertexArrayObject,OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV0});
  Check(std::get<0>(attrib)==3&&std::get<2>(attrib)==24&&std::get<3>(attrib)==12,"UVQ lost third component or interleaved layout");
  const auto* tex=static_cast<const OVRFW::GlTexture*>(surface->graphicsCommand.UniformData[0].Data);
  Check(tex&&mock_gl::texture_data.at(tex->texture)==std::vector<std::uint8_t>({1,2,3,4}),"R8 atlas missing or mixed");
  Check(mock_gl::State()==initial,"successful upload leaked GL state");
  mock_gl::fail_texture=true;Check(!r.Upload(Snapshot(3,true)),"failed texture upload was accepted");
  Check(Draw(r).back().surface==surface,"partial upload replaced the active scene");
  Check(mock_gl::State()==initial,"failed upload leaked GL state");
  mock_gl::fail_texture=false;Check(r.Upload(Snapshot(3,true)),"same publication could not retry after upload failure");
  Check(Draw(r).back().surface!=surface,"successful retry did not publish inactive slot");
  const auto* held=Draw(r).back().surface;
  auto invalid=Snapshot(4,true);auto invalid_frame=std::make_shared<RemoteSceneFrame>(*invalid.frame);
  invalid_frame->indices[2]=99;invalid.frame=invalid_frame;
  Check(!r.Upload(invalid)&&Draw(r).back().surface==held,"out-of-range index reached GPU publication");
  auto raw=Snapshot(4,true);auto raw_frame=std::make_shared<RemoteSceneFrame>(*raw.frame);
  raw_frame->representation=RemoteSceneRepresentation::Unprepared;raw.frame=raw_frame;
  Check(r.Upload(raw)&&Draw(r).back().surface->geo.primitiveType==GL_TRIANGLES,"converted unprepared frame did not use mesh path");
  held=Draw(r).back().surface;
  mock_gl::fail_buffer=true;Check(!r.Upload(Snapshot(5,false)),"point buffer failure was accepted");
  Check(Draw(r).back().surface==held&&mock_gl::State()==initial,"point failure exposed partial state");
  mock_gl::fail_buffer=false;
  Check(r.Upload(Snapshot(5,false)),"point mode failed after texture mode");
  Check(Draw(r).back().surface->geo.primitiveType==GL_POINTS,"point mode retained mesh primitive");
  RemoteSceneSnapshot empty;empty.stream_generation=1;empty.publication=6;
  Check(r.Upload(empty)&&Draw(r).size()==1,"empty source retained stale geometry");
  auto rgb=Snapshot(7,true);auto rgb_frame=std::make_shared<RemoteSceneFrame>(*rgb.frame);
  rgb_frame->wire_version=3;rgb_frame->atlas_channels=3;rgb_frame->map_generation=1;rgb_frame->retention_ms=3000;rgb_frame->map_flags=1;
  rgb_frame->produced_ns=3000000000ULL;
  rgb_frame->observations={{0,2,1,1000000000ULL,1000000000ULL,0,3,0,3,0,0,2,2,3}};
  for(const auto& v:rgb_frame->vertices)rgb_frame->rgb_vertices.push_back({v.position,v.uvq,{.25F,.25F,.75F,.75F},1});
  rgb_frame->vertices.clear();rgb_frame->atlas={255,0,0,0,255,0,0,0,255,200,100,50};rgb.frame=rgb_frame;
  rgb.received_at=std::chrono::steady_clock::now();
  Check(r.Upload(rgb),"RGB surface upload failed");
  const auto* rgb_surface=Draw(r).back().surface;const auto* rgb_tex=static_cast<const OVRFW::GlTexture*>(rgb_surface->graphicsCommand.UniformData[0].Data);
  Check(mock_gl::texture_data.at(rgb_tex->texture)==rgb_frame->atlas,"RGB channel data was lost");
  const auto rgb_attr=mock_gl::attributes.at({rgb_surface->geo.vertexArrayObject,OVRFW::VERTEX_ATTRIBUTE_LOCATION_COLOR});
  Check(std::get<0>(rgb_attr)==2&&std::get<2>(rgb_attr)==48&&std::get<3>(rgb_attr)==40,"RGB validity and observation age layout incorrect");
  std::vector<OVRFW::ovrDrawSurface> aged;
  Check(r.Append(Mat4::Identity(),2000,true,true,aged,rgb.received_at+std::chrono::milliseconds(500)),"live observation expired too early");
  const auto* timing=static_cast<const float*>(aged.back().surface->graphicsCommand.UniformData[1].Data);
  Check(timing[0]==.5F&&timing[1]>2.99F&&timing[1]<3.01F,"local receipt age or retention duration missing");
  aged.clear();Check(!r.Append(Mat4::Identity(),2000,false,true,aged,rgb.received_at+std::chrono::milliseconds(1100))&&aged.size()==1,
                    "disconnected live observations did not expire by original exposure age");
  rgb_frame->flags=kRemoteSceneRecorded;rgb.publication=8;
  Check(r.Upload(rgb),"recorded RGB frame upload failed");aged.clear();
  Check(r.Append(Mat4::Identity(),2000,false,true,aged,rgb.received_at+std::chrono::hours(1)),"recorded inspection must freeze observation ages");
  const auto held_rgb=aged.back().surface;
  mock_gl::fail_texture=true;rgb.publication=9;Check(!r.Upload(rgb),"RGB failed texture upload accepted");mock_gl::fail_texture=false;
  Check(Draw(r).back().surface==held_rgb&&mock_gl::State()==initial,"RGB partial upload exposed a new slot or leaked state");
  Check(r.Upload(rgb),"RGB upload could not retry");
  auto stalled=std::make_shared<RemoteSceneFrame>(*rgb_frame);stalled->flags=0;stalled->observations[0].flags=0;
  stalled->observations[0].depth_capture_ns=stalled->produced_ns;stalled->observations[0].image_capture_ns=1;
  for(auto& v:stalled->rgb_vertices)v.texture_valid=0;
  rgb.frame=stalled;rgb.publication=10;rgb.received_at=std::chrono::steady_clock::now();
  Check(r.Upload(rgb),"fresh depth with stalled unused RGB rejected by renderer");aged.clear();
  Check(r.Append(Mat4::Identity(),2000,false,true,aged,rgb.received_at+std::chrono::seconds(2)),"unused RGB timestamp expired fresh depth geometry");
  r.Shutdown();r.Shutdown();
  Check(mock_gl::vaos.empty()&&mock_gl::buffers.empty()&&mock_gl::texture_ids.empty()&&mock_gl::programs.empty(),"session teardown leaked renderer resources");
  Check(mock_gl::State()==initial,"Shutdown unbound unrelated GL program or buffers");
  after_shutdown=mock_gl::calls;
 }
 Check(mock_gl::calls==after_shutdown,"destructor called GL after explicit session teardown");
 std::cout<<"remote renderer atomic slots, UVQ layout, state restoration and lifetime passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
