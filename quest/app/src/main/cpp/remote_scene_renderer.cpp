#include "remote_scene_renderer.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace quest_newton {
bool RemoteSceneProjectionScale(const Mat4& map,float viewport_height,float& output){
    if(!std::isfinite(viewport_height)||viewport_height<=0)return false;
    for(float value:map.m)if(!std::isfinite(value))return false;
    if(std::abs(map.m[12])>1e-6F||std::abs(map.m[13])>1e-6F||std::abs(map.m[14])>1e-6F||std::abs(map.m[15]-1.F)>1e-6F)return false;
    std::array<std::array<double,3>,3> columns{};
    std::array<double,3> lengths{};
    for(std::size_t col=0;col<3;++col){
        for(std::size_t row=0;row<3;++row){columns[col][row]=map.m[row*4+col];lengths[col]+=columns[col][row]*columns[col][row];}
        lengths[col]=std::sqrt(lengths[col]);if(lengths[col]<=0)return false;
    }
    for(std::size_t col=1;col<3;++col)if(std::abs(lengths[col]-lengths[0])>lengths[0]*1e-4)return false;
    for(std::size_t a=0;a<3;++a)for(std::size_t b=a+1;b<3;++b){
        double dot=0;for(std::size_t row=0;row<3;++row)dot+=columns[a][row]*columns[b][row];
        if(std::abs(dot)>lengths[a]*lengths[b]*1e-4)return false;
    }
    const auto& a=columns[0];const auto& b=columns[1];const auto& c=columns[2];
    if(a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])<=0)return false;
    const double value=viewport_height*lengths[0];
    if(!std::isfinite(value)||value>std::numeric_limits<float>::max())return false;
    output=static_cast<float>(value);return output>0;
}
}

#if defined(__ANDROID__)
#include <GLES3/gl3.h>
#include "Render/SurfaceRender.h"
#include <cstddef>
#include <numeric>

namespace quest_newton {
namespace {
// GlProgram binds these names to the SDK's fixed attribute locations. The
// point VBO is our explicit interleaved layout, not VertexAttribs' packed arrays.
static_assert(sizeof(RemoteScenePoint)==16);
static_assert(offsetof(RemoteScenePoint,position)==0&&offsetof(RemoteScenePoint,grayscale)==12&&offsetof(RemoteScenePoint,radius_mm)==14);
static_assert(kRemoteSceneMaxPoints<=OVRFW::GlGeometry::kMaxGeometryVertices);
static_assert(sizeof(RemoteSceneVertex)==24&&offsetof(RemoteSceneVertex,uvq)==12);
static_assert(kRemoteSurfaceMaxVertices<=OVRFW::GlGeometry::kMaxGeometryVertices);
struct RgbGpuVertex {RemoteSceneRgbVertex vertex;float age_seconds=0;};
static_assert(sizeof(RgbGpuVertex)==48);
constexpr auto kVertexCapacity=std::max(kRemoteSceneMaxPoints*sizeof(RemoteScenePoint),kRemoteRgbMaxVertices*sizeof(RgbGpuVertex));
constexpr auto kIndexCapacity=std::max(kRemoteSceneMaxPoints,kRemoteRgbMaxIndices);
OVR::Matrix4f Matrix(const Mat4& matrix){
    OVR::Matrix4f result;
    for(std::size_t row=0;row<4;++row)for(std::size_t col=0;col<4;++col)result.M[row][col]=matrix.m[row*4+col];
    return result;
}
struct Bindings {
    GLint vao=0,array=0,element=0,program=0,active=0,texture=0,unpack_buffer=0;
    GLint alignment=0,row_length=0,skip_rows=0,skip_pixels=0;
    Bindings(){
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING,&vao);glGetIntegerv(GL_ARRAY_BUFFER_BINDING,&array);
        glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING,&element);glGetIntegerv(GL_CURRENT_PROGRAM,&program);
        glGetIntegerv(GL_ACTIVE_TEXTURE,&active);glActiveTexture(GL_TEXTURE0);
        glGetIntegerv(GL_TEXTURE_BINDING_2D,&texture);
        glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING,&unpack_buffer);
        glGetIntegerv(GL_UNPACK_ALIGNMENT,&alignment);glGetIntegerv(GL_UNPACK_ROW_LENGTH,&row_length);
        glGetIntegerv(GL_UNPACK_SKIP_ROWS,&skip_rows);glGetIntegerv(GL_UNPACK_SKIP_PIXELS,&skip_pixels);
    }
    ~Bindings(){
        glBindVertexArray(static_cast<GLuint>(vao));glBindBuffer(GL_ARRAY_BUFFER,static_cast<GLuint>(array));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,static_cast<GLuint>(element));glUseProgram(static_cast<GLuint>(program));
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER,static_cast<GLuint>(unpack_buffer));
        glPixelStorei(GL_UNPACK_ALIGNMENT,alignment);glPixelStorei(GL_UNPACK_ROW_LENGTH,row_length);
        glPixelStorei(GL_UNPACK_SKIP_ROWS,skip_rows);glPixelStorei(GL_UNPACK_SKIP_PIXELS,skip_pixels);
        glActiveTexture(GL_TEXTURE0);glBindTexture(GL_TEXTURE_2D,static_cast<GLuint>(texture));
        glActiveTexture(static_cast<GLenum>(active));
    }
};
GLenum TakeError(){
    const auto first=glGetError();
    if(first!=GL_NO_ERROR)for(unsigned i=0;i<16&&glGetError()!=GL_NO_ERROR;++i){}
    return first;
}
void CreateGeometry(OVRFW::GlGeometry& geo,std::size_t vertex_bytes,std::size_t index_count){
    glGenVertexArrays(1,&geo.vertexArrayObject);glGenBuffers(1,&geo.vertexBuffer);glGenBuffers(1,&geo.indexBuffer);
    glBindVertexArray(geo.vertexArrayObject);glBindBuffer(GL_ARRAY_BUFFER,geo.vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(vertex_bytes),nullptr,GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,geo.indexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,static_cast<GLsizeiptr>(index_count*sizeof(OVRFW::TriangleIndex)),nullptr,GL_DYNAMIC_DRAW);
    geo.indexType=GL_UNSIGNED_SHORT;geo.vertexCount=geo.indexCount=0;
    geo.localBounds.Clear();geo.localBounds.AddPoint(OVR::Vector3f{-100,-100,-100});geo.localBounds.AddPoint(OVR::Vector3f{100,100,100});
}
bool FitsGpu(const RemoteSceneFrame& frame,GLint max_texture){
    if(frame.wire_version<1||frame.wire_version>3)return false;
    if(frame.wire_version==3){
        if((frame.representation!=RemoteSceneRepresentation::Prepared&&frame.representation!=RemoteSceneRepresentation::Unprepared)||
           !frame.points.empty()||!frame.vertices.empty()||frame.rgb_vertices.size()>kRemoteRgbMaxVertices||frame.indices.size()>kRemoteRgbMaxIndices||frame.indices.size()%3||
           frame.atlas_channels!=3||frame.observations.size()>10||frame.retention_ms<1||frame.retention_ms>60000)return false;
        if(frame.atlas_width>2048||frame.atlas_height>2048||frame.atlas_width>static_cast<std::uint32_t>(max_texture)||frame.atlas_height>static_cast<std::uint32_t>(max_texture))return false;
        const auto pixels=static_cast<std::size_t>(frame.atlas_width)*frame.atlas_height;
        if(pixels>kRemoteRgbMaxPixels||pixels*3!=frame.atlas.size())return false;
        std::uint64_t vertices=0,indices=0;
        for(const auto& o:frame.observations){
            if(o.vertex_start!=vertices||o.index_start!=indices||!o.depth_capture_ns||!o.image_capture_ns||
               o.depth_capture_ns>frame.produced_ns||o.image_capture_ns>frame.produced_ns)return false;
            vertices+=o.vertex_count;indices+=o.index_count;
        }
        return vertices==frame.rgb_vertices.size()&&indices==frame.indices.size()&&
            (frame.observations.empty()?pixels==0:pixels>0)&&
            std::all_of(frame.indices.begin(),frame.indices.end(),[&](auto i){return i<vertices;});
    }
    if(frame.representation==RemoteSceneRepresentation::Points)
        return frame.points.size()<=kRemoteSceneMaxPoints&&frame.vertices.empty()&&frame.indices.empty()&&
               frame.atlas.empty()&&!frame.atlas_width&&!frame.atlas_height;
    if(frame.representation!=RemoteSceneRepresentation::Prepared&&frame.representation!=RemoteSceneRepresentation::Unprepared)return false;
    if(!frame.points.empty()||frame.vertices.size()>kRemoteSurfaceMaxVertices||frame.indices.size()>kRemoteSurfaceMaxIndices||frame.indices.size()%3)return false;
    if(frame.indices.empty())return frame.vertices.empty()&&frame.atlas.empty()&&!frame.atlas_width&&!frame.atlas_height;
    if(frame.vertices.empty()||!frame.atlas_width||!frame.atlas_height||
       frame.atlas_width>kRemoteSurfaceMaxAtlasDimension||frame.atlas_height>kRemoteSurfaceMaxAtlasDimension||
       frame.atlas_width>static_cast<std::uint32_t>(max_texture)||frame.atlas_height>static_cast<std::uint32_t>(max_texture))return false;
    const auto pixels=static_cast<std::size_t>(frame.atlas_width)*frame.atlas_height;
    return pixels<=kRemoteSurfaceMaxAtlasPixels&&pixels==frame.atlas.size()&&
           std::all_of(frame.indices.begin(),frame.indices.end(),[&](auto index){return index<frame.vertices.size();});
}
constexpr const char* kPointVertex=R"glsl(
attribute highp vec4 Position;
attribute lowp vec4 VertexColor;
attribute highp vec2 TexCoord;
uniform highp vec4 PointParameters;
varying lowp float PointGray;
void main(){
    gl_Position=TransformVertex(Position);
    highp float diameter=TexCoord.x*0.001*PointParameters.x*abs(sm.ProjectionMatrix[VIEW_ID][1][1])/max(gl_Position.w,0.001);
    gl_PointSize=clamp(diameter,PointParameters.y,PointParameters.z);
    PointGray=VertexColor.x;
})glsl";
constexpr const char* kPointFragment=R"glsl(
precision mediump float;
varying lowp float PointGray;
uniform lowp vec4 Tint;
void main(){
    vec2 offset=gl_PointCoord*2.0-1.0;
    if(dot(offset,offset)>1.0)discard;
    gl_FragColor=vec4(vec3(PointGray)*Tint.rgb,1.0);
})glsl";
constexpr const char* kMeshVertex=R"glsl(
attribute highp vec4 Position;
attribute highp vec3 TexCoord;
varying highp vec3 ImageUVQ;
void main(){
    gl_Position=TransformVertex(Position);
    ImageUVQ=TexCoord;
})glsl";
constexpr const char* kMeshFragment=R"glsl(
precision highp float;
varying highp vec3 ImageUVQ;
uniform lowp sampler2D Texture0;
uniform lowp vec4 Tint;
void main(){
    if(ImageUVQ.z<=0.0)discard;
    highp vec2 uv=ImageUVQ.xy/ImageUVQ.z;
    lowp float gray=texture2D(Texture0,uv).r;
    gl_FragColor=vec4(vec3(gray)*Tint.rgb,1.0);
})glsl";
constexpr const char* kBlackVertex="attribute highp vec4 Position; void main(){gl_Position=vec4(Position.xy,1.0,1.0);}";
constexpr const char* kBlackFragment="precision mediump float; void main(){gl_FragColor=vec4(0.0,0.0,0.0,1.0);}";
constexpr const char* kRgbVertex=R"glsl(
attribute highp vec4 Position;
attribute highp vec3 TexCoord;
attribute highp vec4 TexCoord1;
attribute highp vec2 VertexColor;
varying highp vec3 ImageUVQ;
varying highp vec4 TileBounds;
varying highp vec2 ValidityAge;
void main(){gl_Position=TransformVertex(Position);ImageUVQ=TexCoord;TileBounds=TexCoord1;ValidityAge=VertexColor;}
)glsl";
constexpr const char* kRgbFragment=R"glsl(
precision highp float;
varying highp vec3 ImageUVQ;
varying highp vec4 TileBounds;
varying highp vec2 ValidityAge;
uniform lowp sampler2D Texture0;
uniform highp vec4 ObservationTiming;
void main(){
    if(ValidityAge.y+ObservationTiming.x>ObservationTiming.y)discard;
    lowp vec3 color=vec3(128.0/255.0);
    if(ImageUVQ.z>0.0&&ValidityAge.x>=1.0-1e-6){
        highp vec2 uv=ImageUVQ.xy/ImageUVQ.z;
        if(all(greaterThanEqual(uv,TileBounds.xy))&&all(lessThanEqual(uv,TileBounds.zw)))color=texture2D(Texture0,uv).rgb;
    }
    gl_FragColor=vec4(color,1.0);
})glsl";
}
struct RemoteSceneRenderer::Impl {
    struct Slot {OVRFW::ovrSurfaceDef surface;OVRFW::GlTexture atlas;bool rgb=false,freeze_age=false;RemoteSceneTime received_at{};float retention_seconds=0,youngest_age=0;};
    OVRFW::GlProgram point_program,mesh_program,rgb_program,black_program;
    std::array<Slot,2> slots;
    OVRFW::ovrSurfaceDef black;
    std::array<OVRFW::TriangleIndex,kRemoteSceneMaxPoints> point_indices{};
    std::array<float,4> point_parameters{1,1,64,0},tint{1,1,1,1};
    std::array<float,4> observation_timing{};
    GLint max_texture=0;
    int active=-1;
    bool ready=false;
    std::uint64_t generation=0,publication=0;
    std::string error;
};
RemoteSceneRenderer::RemoteSceneRenderer():impl_(std::make_unique<Impl>()){}
// GL cleanup is explicit: destruction can occur after XrApp destroyed EGL.
RemoteSceneRenderer::~RemoteSceneRenderer()=default;
bool RemoteSceneRenderer::Init(){
    auto& p=*impl_;if(p.ready)return true;
    const auto fail=[&](std::string message){Shutdown();p.error=std::move(message);return false;};
    Bindings bindings;
    if(TakeError()!=GL_NO_ERROR)return fail("Remote renderer entered initialization with a GL error");
    const OVRFW::ovrProgramParm uniforms[]={{"PointParameters",OVRFW::ovrProgramParmType::FLOAT_VECTOR4},{"Tint",OVRFW::ovrProgramParmType::FLOAT_VECTOR4}};
    const OVRFW::ovrProgramParm mesh_uniforms[]={{"Texture0",OVRFW::ovrProgramParmType::TEXTURE_SAMPLED},{"Tint",OVRFW::ovrProgramParmType::FLOAT_VECTOR4}};
    const OVRFW::ovrProgramParm rgb_uniforms[]={{"Texture0",OVRFW::ovrProgramParmType::TEXTURE_SAMPLED},{"ObservationTiming",OVRFW::ovrProgramParmType::FLOAT_VECTOR4}};
    p.point_program=OVRFW::GlProgram::Build(kPointVertex,kPointFragment,uniforms,2,OVRFW::GlProgram::GLSL_PROGRAM_VERSION,false);
    p.mesh_program=OVRFW::GlProgram::Build(kMeshVertex,kMeshFragment,mesh_uniforms,2,OVRFW::GlProgram::GLSL_PROGRAM_VERSION,false);
    p.rgb_program=OVRFW::GlProgram::Build(kRgbVertex,kRgbFragment,rgb_uniforms,2,OVRFW::GlProgram::GLSL_PROGRAM_VERSION,false);
    p.black_program=OVRFW::GlProgram::Build(kBlackVertex,kBlackFragment,nullptr,0,OVRFW::GlProgram::GLSL_PROGRAM_VERSION,false);
    if(!p.point_program.IsValid()||!p.mesh_program.IsValid()||!p.rgb_program.IsValid()||!p.black_program.IsValid())return fail("Remote shader initialization failed");
    GLfloat point_range[2]{};glGetFloatv(GL_ALIASED_POINT_SIZE_RANGE,point_range);
    if(!std::isfinite(point_range[0])||!std::isfinite(point_range[1])||point_range[0]<=0||point_range[0]>64||point_range[1]<point_range[0])
        return fail("Remote GL point-size range is unavailable");
    p.point_parameters[1]=point_range[0];p.point_parameters[2]=std::min(64.F,point_range[1]);
    glGetIntegerv(GL_MAX_TEXTURE_SIZE,&p.max_texture);
    GLint cull_mode=0;glGetIntegerv(GL_CULL_FACE_MODE,&cull_mode);
    // The SDK controls front-face winding and cull enable, but has no cull-mode
    // field. It uses the context's unchanged GL_BACK default throughout.
    if(p.max_texture<=0||cull_mode!=GL_BACK)return fail("Remote texture or backface state is unavailable");
    std::iota(p.point_indices.begin(),p.point_indices.end(),OVRFW::TriangleIndex{0});
    for(auto& slot:p.slots){
        CreateGeometry(slot.surface.geo,kVertexCapacity,kIndexCapacity);
        if(!slot.surface.geo.vertexArrayObject||!slot.surface.geo.vertexBuffer||!slot.surface.geo.indexBuffer)
            return fail("Remote geometry buffers are unavailable");
        auto& state=slot.surface.graphicsCommand.GpuState;
        state.blendEnable=OVRFW::ovrGpuState::BLEND_DISABLE;state.depthEnable=true;state.depthMaskEnable=true;
        state.depthFunc=GL_LEQUAL;state.frontFace=GL_CCW;
    }
    auto& geo=p.black.geo;
    constexpr std::array<float,6> triangle{-1,-1,3,-1,-1,3};
    constexpr std::array<OVRFW::TriangleIndex,3> triangle_indices{0,1,2};
    CreateGeometry(geo,sizeof(triangle),triangle_indices.size());
    glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(triangle),triangle.data());
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,sizeof(triangle_indices),triangle_indices.data());
    glEnableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_POSITION);
    glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_POSITION,2,GL_FLOAT,GL_FALSE,2*sizeof(float),nullptr);
    geo.primitiveType=GL_TRIANGLES;geo.vertexCount=geo.indexCount=3;
    p.black.surfaceName="remote_inspection_black_background";
    auto& black_command=p.black.graphicsCommand;black_command.Program=p.black_program;
    black_command.GpuState.blendEnable=OVRFW::ovrGpuState::BLEND_DISABLE;
    black_command.GpuState.depthEnable=false;black_command.GpuState.depthMaskEnable=false;black_command.GpuState.cullEnable=false;
    if(TakeError()!=GL_NO_ERROR||!geo.vertexArrayObject||!geo.vertexBuffer||!geo.indexBuffer)
        return fail("Remote buffer initialization failed");
    p.active=-1;p.generation=p.publication=0;p.ready=true;p.error.clear();return true;
}
bool RemoteSceneRenderer::Upload(const RemoteSceneSnapshot& snapshot){
    auto& p=*impl_;
    if(!p.ready){p.error="Remote renderer is not initialized";return false;}
    if(snapshot.stream_generation==p.generation&&snapshot.publication==p.publication)return true;
    if(!snapshot.frame){p.active=-1;p.generation=snapshot.stream_generation;p.publication=snapshot.publication;p.error.clear();return true;}
    if(!snapshot.stream_generation||!snapshot.publication||!FitsGpu(*snapshot.frame,p.max_texture)){
        p.error="Invalid remote frame publication or geometry bounds";return false;
    }
    const auto& frame=*snapshot.frame;
    if(!frame.HasGeometry()){p.active=-1;p.generation=snapshot.stream_generation;p.publication=snapshot.publication;p.error.clear();return true;}
    const bool points=frame.representation==RemoteSceneRepresentation::Points;
    const bool rgb=frame.wire_version==3;
    std::vector<RgbGpuVertex> rgb_staging;
    float youngest_age=std::numeric_limits<float>::infinity();
    if(rgb){
        rgb_staging.resize(frame.rgb_vertices.size());
        for(const auto& o:frame.observations){
            const float age=static_cast<float>(static_cast<double>(frame.produced_ns-RemoteObservationCaptureNs(o))/1e9);
            if(o.index_count)youngest_age=std::min(youngest_age,age);
            for(std::size_t i=o.vertex_start;i<static_cast<std::size_t>(o.vertex_start)+o.vertex_count;++i)rgb_staging[i]={frame.rgb_vertices[i],age};
        }
    }
    const int next=p.active==0?1:0;
    auto& slot=p.slots[static_cast<std::size_t>(next)];auto& geo=slot.surface.geo;
    {
        Bindings bindings;
        if(TakeError()!=GL_NO_ERROR){p.error="Remote upload began with a pending GL error";return false;}
        glBindVertexArray(geo.vertexArrayObject);glBindBuffer(GL_ARRAY_BUFFER,geo.vertexBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,geo.indexBuffer);
        glEnableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_POSITION);
        glEnableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV0);
        glDisableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV1);
        if(points){
            glBufferSubData(GL_ARRAY_BUFFER,0,static_cast<GLsizeiptr>(frame.points.size()*sizeof(RemoteScenePoint)),frame.points.data());
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,static_cast<GLsizeiptr>(frame.points.size()*sizeof(OVRFW::TriangleIndex)),p.point_indices.data());
            glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_POSITION,3,GL_FLOAT,GL_FALSE,sizeof(RemoteScenePoint),nullptr);
            glEnableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_COLOR);
            glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_COLOR,1,GL_UNSIGNED_BYTE,GL_TRUE,sizeof(RemoteScenePoint),
                reinterpret_cast<void*>(offsetof(RemoteScenePoint,grayscale)));
            glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV0,1,GL_UNSIGNED_SHORT,GL_FALSE,sizeof(RemoteScenePoint),
                reinterpret_cast<void*>(offsetof(RemoteScenePoint,radius_mm)));
        }else{
            glBufferSubData(GL_ARRAY_BUFFER,0,static_cast<GLsizeiptr>(rgb?rgb_staging.size()*sizeof(RgbGpuVertex):frame.vertices.size()*sizeof(RemoteSceneVertex)),
                rgb?static_cast<const void*>(rgb_staging.data()):static_cast<const void*>(frame.vertices.data()));
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,static_cast<GLsizeiptr>(frame.indices.size()*sizeof(std::uint16_t)),frame.indices.data());
            glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_POSITION,3,GL_FLOAT,GL_FALSE,rgb?sizeof(RgbGpuVertex):sizeof(RemoteSceneVertex),nullptr);
            glDisableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_COLOR);
            glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV0,3,GL_FLOAT,GL_FALSE,rgb?sizeof(RgbGpuVertex):sizeof(RemoteSceneVertex),
                reinterpret_cast<void*>(offsetof(RemoteSceneVertex,uvq)));
            if(rgb){
                glEnableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV1);glEnableVertexAttribArray(OVRFW::VERTEX_ATTRIBUTE_LOCATION_COLOR);
                glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_UV1,4,GL_FLOAT,GL_FALSE,sizeof(RgbGpuVertex),reinterpret_cast<void*>(24));
                glVertexAttribPointer(OVRFW::VERTEX_ATTRIBUTE_LOCATION_COLOR,2,GL_FLOAT,GL_FALSE,sizeof(RgbGpuVertex),reinterpret_cast<void*>(40));
            }
            if(!slot.atlas.texture)glGenTextures(1,&slot.atlas.texture);
            glBindTexture(GL_TEXTURE_2D,slot.atlas.texture);
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER,0);glPixelStorei(GL_UNPACK_ALIGNMENT,1);
            glPixelStorei(GL_UNPACK_ROW_LENGTH,0);glPixelStorei(GL_UNPACK_SKIP_ROWS,0);glPixelStorei(GL_UNPACK_SKIP_PIXELS,0);
            // Redefine only the inactive slot's bounded storage. No SDK texture
            // loader is used: it drains GL errors without returning upload status.
            glTexImage2D(GL_TEXTURE_2D,0,rgb?GL_RGB8:GL_R8,static_cast<GLsizei>(frame.atlas_width),static_cast<GLsizei>(frame.atlas_height),
                0,rgb?GL_RGB:GL_RED,GL_UNSIGNED_BYTE,frame.atlas.data());
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        }
        if(TakeError()!=GL_NO_ERROR||(!points&&!slot.atlas.texture)){
            p.error="Remote geometry/atlas upload failed; previous GPU publication retained";return false;
        }
    }
    // Commit all draw-facing metadata only after every inactive-slot GL upload
    // succeeds. Neither a failure nor a mode switch can expose mixed buffers.
    auto& command=slot.surface.graphicsCommand;
    geo.primitiveType=points?GL_POINTS:GL_TRIANGLES;
    geo.vertexCount=static_cast<std::int32_t>(points?frame.points.size():rgb?frame.rgb_vertices.size():frame.vertices.size());
    geo.indexCount=static_cast<std::int32_t>(points?frame.points.size():frame.indices.size());
    command.Program=points?p.point_program:rgb?p.rgb_program:p.mesh_program;command.GpuState.cullEnable=!points;
    slot.surface.surfaceName=points?"remote_observed_grayscale_points":rgb?"remote_observed_rgb_surface":"remote_observed_textured_surface";
    if(points)command.UniformData[0].Data=p.point_parameters.data();
    else{
        slot.atlas.target=GL_TEXTURE_2D;slot.atlas.Width=static_cast<int>(frame.atlas_width);slot.atlas.Height=static_cast<int>(frame.atlas_height);
        command.UniformData[0].Data=&slot.atlas;
    }
    command.UniformData[1].Data=rgb?p.observation_timing.data():p.tint.data();
    slot.rgb=rgb;slot.received_at=snapshot.received_at;slot.freeze_age=(frame.flags&(kRemoteSceneRecorded|kRemoteSceneSynthetic))!=0;
    slot.retention_seconds=static_cast<float>(frame.retention_ms)*.001F;slot.youngest_age=youngest_age;p.active=next;
    p.generation=snapshot.stream_generation;p.publication=snapshot.publication;p.error.clear();return true;
}
bool RemoteSceneRenderer::Append(const Mat4& map,float viewport_height,bool stale,bool black_background,std::vector<OVRFW::ovrDrawSurface>& surfaces,RemoteSceneTime now){
    auto& p=*impl_;if(!p.ready)return false;
    if(black_background)surfaces.emplace_back(OVR::Matrix4f::Identity(),&p.black);
    if(p.active<0)return false;
    const auto& slot=p.slots[static_cast<std::size_t>(p.active)];
    if(slot.rgb){
        if(!slot.freeze_age&&now<slot.received_at){p.error="Remote observation receipt clock is invalid";return false;}
        const auto age=slot.freeze_age?0.F:static_cast<float>(std::chrono::duration<double>(now-slot.received_at).count());
        p.observation_timing={age,slot.retention_seconds,0,0};
        if(slot.youngest_age+age>slot.retention_seconds)return false;
    }
    if(!RemoteSceneProjectionScale(map,viewport_height,p.point_parameters[0])){p.error="Remote inspection transform or viewport is invalid";return false;}
    p.tint=stale?std::array<float,4>{1.F,.6F,.2F,1.F}:std::array<float,4>{1,1,1,1};
    surfaces.emplace_back(Matrix(map),&p.slots[static_cast<std::size_t>(p.active)].surface);
    return true;
}
void RemoteSceneRenderer::Shutdown(){
    auto& p=*impl_;
    const bool allocated=p.point_program.IsValid()||p.mesh_program.IsValid()||p.rgb_program.IsValid()||p.black_program.IsValid()||p.black.geo.vertexArrayObject||
        std::any_of(p.slots.begin(),p.slots.end(),[](const auto& slot){return slot.surface.geo.vertexArrayObject||slot.surface.geo.vertexBuffer||slot.surface.geo.indexBuffer||slot.atlas.texture;});
    if(allocated){
        GLint program=0;glGetIntegerv(GL_CURRENT_PROGRAM,&program);
        const bool own_program=static_cast<GLuint>(program)==p.point_program.Program||static_cast<GLuint>(program)==p.mesh_program.Program||static_cast<GLuint>(program)==p.rgb_program.Program||static_cast<GLuint>(program)==p.black_program.Program;
        for(auto& slot:p.slots){slot.surface.geo.Free();OVRFW::FreeTexture(slot.atlas);slot.atlas={};slot.surface.graphicsCommand.Program={};}
        p.black.geo.Free();OVRFW::GlProgram::Free(p.point_program);OVRFW::GlProgram::Free(p.mesh_program);OVRFW::GlProgram::Free(p.rgb_program);OVRFW::GlProgram::Free(p.black_program);
        // SDK Free unbinds the current program even if it was unrelated.
        if(!own_program)glUseProgram(static_cast<GLuint>(program));
    }
    p.point_program={};p.mesh_program={};p.rgb_program={};p.black_program={};p.active=-1;p.ready=false;p.generation=p.publication=0;p.error.clear();
}
bool RemoteSceneRenderer::Ready()const{return impl_->ready;}
const std::string& RemoteSceneRenderer::Error()const{return impl_->error;}
} // namespace quest_newton
#endif
