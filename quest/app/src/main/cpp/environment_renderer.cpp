#include "environment_renderer.h"
#include "room_environment.h"
#include "Render/GlGeometry.h"
#include <GLES3/gl3.h>
#include <algorithm>
#include <cmath>

namespace quest_newton {
namespace {
OVR::Matrix4f Matrix(const Mat4& matrix){
    OVR::Matrix4f result;for(std::size_t r=0;r<4;++r)for(std::size_t c=0;c<4;++c)result.M[r][c]=matrix.m[r*4+c];return result;
}
}
bool EnvironmentRenderer::Init(){
    if(ready_)return true;
    const char* vertex="attribute highp vec4 Position; void main(){ gl_Position = TransformVertex(Position); }";
    const char* fragment="precision mediump float; uniform lowp vec4 Color; void main(){ gl_FragColor = Color; }";
    const OVRFW::ovrProgramParm uniforms[]={{"Color",OVRFW::ovrProgramParmType::FLOAT_VECTOR4}};
    program_=OVRFW::GlProgram::Build(vertex,fragment,uniforms,1,OVRFW::GlProgram::GLSL_PROGRAM_VERSION,false);
    if(!program_.IsValid())return false;
    OVRFW::VertexAttribs vertices;std::vector<OVRFW::TriangleIndex> indices;
    const auto strip=[&](float x0,float y0,float x1,float y1){
        const auto first=static_cast<OVRFW::TriangleIndex>(vertices.position.size());
        vertices.position.emplace_back(x0,y0,0);vertices.position.emplace_back(x1,y0,0);
        vertices.position.emplace_back(x1,y1,0);vertices.position.emplace_back(x0,y1,0);
        for(const unsigned index:{0U,1U,2U,0U,2U,3U})indices.push_back(static_cast<OVRFW::TriangleIndex>(first+index));
    };
    for(int i=-8;i<=8;++i){const float coordinate=static_cast<float>(i)*.25F;
        const float width=i==0?.006F:.002F;strip(coordinate-width,-2,coordinate+width,2);strip(-2,coordinate-width,2,coordinate+width);}
    {
        const OVRFW::GlGeometry::TransformScope no_transform(OVR::Matrix4f::Identity(),false);
        grid_.geo.Create(vertices,indices);
        const auto cube=OVRFW::BuildUnitCubeDescriptor(1.F);box_.geo.Create(cube.attribs,cube.indices);
        OVRFW::VertexAttribs corners;
        for (const float z:{-1.F,1.F}) for (const float y:{-1.F,1.F}) for (const float x:{-1.F,1.F})
            corners.position.emplace_back(x,y,z);
        const std::vector<OVRFW::TriangleIndex> edges{0,1,1,3,3,2,2,0,4,5,5,7,7,6,6,4,0,4,1,5,2,6,3,7};
        room_.geo.Create(corners,edges);room_.geo.primitiveType=OVRFW::GlGeometry::kPrimitiveTypeLines;
        for(auto* surface:{&target_,&measured_,&preview_,&selected_,&contact_}){
            surface->geo.Create(corners,edges);surface->geo.primitiveType=OVRFW::GlGeometry::kPrimitiveTypeLines;
        }
        OVRFW::VertexAttribs line;line.position.emplace_back(0,0,0);line.position.emplace_back(0,0,1);
        normal_.geo.Create(line,{0,1});normal_.geo.primitiveType=OVRFW::GlGeometry::kPrimitiveTypeLines;
    }
    grid_.surfaceName="simulation_z_zero_grid";box_.surfaceName="dynamic_debug_box";
    room_.surfaceName="estimated_room_surface_outline";
    const auto setup=[&](OVRFW::ovrSurfaceDef& surface,std::array<float,4>& color){
        auto& command=surface.graphicsCommand;command.Program=program_;command.UniformData[0].Data=color.data();
        command.GpuState.blendEnable=OVRFW::ovrGpuState::BLEND_ENABLE;
        command.GpuState.blendSrc=GL_ONE;command.GpuState.blendDst=GL_ONE_MINUS_SRC_ALPHA;
        command.GpuState.depthEnable=true;command.GpuState.depthMaskEnable=false;command.GpuState.depthFunc=GL_LEQUAL;
        command.GpuState.cullEnable=false;
    };
    setup(grid_,grid_color_);setup(box_,box_color_);setup(room_,room_color_);
    setup(target_,target_color_);setup(measured_,measured_color_);setup(preview_,preview_color_);
    setup(selected_,selected_color_);selected_.surfaceName="selected_user_cube";
    setup(contact_,contact_color_);setup(normal_,contact_color_);
    target_.surfaceName="commanded_palm";measured_.surfaceName="measured_palm";preview_.surfaceName="cube_placement_preview";
    contact_.surfaceName="object_contact_point";normal_.surfaceName="contact_normal_force_scaled";
    ready_=glGetError()==GL_NO_ERROR && grid_.geo.vertexBuffer!=0 && box_.geo.vertexBuffer!=0 && room_.geo.vertexBuffer!=0;
    if(!ready_)Shutdown();return ready_;
}
void EnvironmentRenderer::Shutdown(){
    for(auto* surface:{&target_,&measured_,&preview_,&selected_,&contact_,&normal_})surface->geo.Free();
    grid_.geo.Free();box_.geo.Free();room_.geo.Free();OVRFW::GlProgram::Free(program_);program_={};ready_=false;
}
void EnvironmentRenderer::Append(const SceneSnapshot& snapshot,const Mat4& stage_from_world,bool grid,bool room_overlay,
                                 std::vector<OVRFW::ovrDrawSurface>& surfaces) const {
    if(!ready_)return;
    if(grid)surfaces.emplace_back(Matrix(stage_from_world),&grid_);
    for(const auto& object:snapshot.objects){
        if(object.body_index>=snapshot.bodies.size() || object.kind!=1)continue;
        const auto& pose=snapshot.bodies[object.body_index];
        auto model=Multiply(stage_from_world,PoseMatrix({pose.position,pose.rotation}));
        for(std::size_t r=0;r<3;++r)for(std::size_t c=0;c<3;++c)model.m[r*4+c]*=object.half_extents[c];
        surfaces.emplace_back(Matrix(model),&box_);
    }
    if (room_overlay && snapshot.room && snapshot.room->enabled) {
        for (const auto& collider:snapshot.room->colliders) {
            const auto& pose=collider.world_from_collider;
            auto model=Multiply(stage_from_world,PoseMatrix({pose.position,pose.rotation}));
            for (std::size_t r=0;r<3;++r) for (std::size_t c=0;c<3;++c) model.m[r*4+c]*=collider.half_extents[c];
            surfaces.emplace_back(Matrix(model),&room_);
        }
    }
}
void EnvironmentRenderer::AppendDiagnostics(const Mat4& frame,const std::optional<kinematics::Pose>& target,
    const std::optional<kinematics::Pose>& measured,const std::optional<kinematics::Pose>& preview,
    const kinematics::Vec3& half,std::span<const ContactPoint> contacts,const std::optional<CubeMarker>& selected,
    std::vector<OVRFW::ovrDrawSurface>& surfaces) const {
    if(!ready_)return;
    const auto draw=[&](const kinematics::Pose& pose,const kinematics::Vec3& scale,const OVRFW::ovrSurfaceDef& surface){
        auto matrix=Multiply(frame,PoseMatrix({pose.position,pose.rotation}));
        for(std::size_t row=0;row<3;++row)for(std::size_t col=0;col<3;++col)matrix.m[row*4+col]*=scale[col];
        surfaces.emplace_back(Matrix(matrix),&surface);
    };
    if(target)draw(*target,{.012F,.012F,.018F},target_);
    if(measured)draw(*measured,{.009F,.009F,.015F},measured_);
    if(preview)draw(*preview,half,preview_);
    if(selected){auto scale=selected->half_extents;for(auto& value:scale)value*=1.04F;draw(selected->pose,scale,selected_);}
    for(const auto& contact:contacts){
        draw({contact.position,{0,0,0,1}},{.003F,.003F,.003F},contact_);
        kinematics::Pose line{contact.position,{-contact.normal[1],contact.normal[0],0,1+contact.normal[2]}},normalized;
        if(contact.normal[2]<-.9999F)line.rotation={1,0,0,0};
        if(kinematics::NormalizePose(line,normalized))
            draw(normalized,{1,1,.015F+std::min(std::abs(contact.normal_force)*.003F,.1F)},normal_);
    }
}
} // namespace quest_newton
