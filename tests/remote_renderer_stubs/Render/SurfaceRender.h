#pragma once
#include <GLES3/gl3.h>
#include <string>
namespace OVR {
struct Vector3f {float x,y,z;};
struct Matrix4f {float M[4][4]{};static Matrix4f Identity(){Matrix4f m;for(int i=0;i<4;++i)m.M[i][i]=1;return m;}};
struct Bounds3f {void Clear(){} void AddPoint(Vector3f){} };
}
namespace OVRFW {
inline constexpr GLuint VERTEX_ATTRIBUTE_LOCATION_POSITION=0,VERTEX_ATTRIBUTE_LOCATION_COLOR=4,VERTEX_ATTRIBUTE_LOCATION_UV0=5,VERTEX_ATTRIBUTE_LOCATION_UV1=6;
using TriangleIndex=std::uint16_t;
enum class ovrProgramParmType {FLOAT_VECTOR4,TEXTURE_SAMPLED};
struct ovrProgramParm {const char* Name;ovrProgramParmType Type;};
struct GlTexture {GLuint texture=0,target=0;int Width=0,Height=0;
 GlTexture()=default;GlTexture(GLuint id,GLuint t,int w,int h):texture(id),target(t),Width(w),Height(h){};};
inline void FreeTexture(GlTexture t){glDeleteTextures(1,&t.texture);}
struct GlProgram {GLuint Program=0;static constexpr int GLSL_PROGRAM_VERSION=300;
 static GlProgram Build(const char*,const char*,const ovrProgramParm*,int,int,bool){
  GlProgram p{mock_gl::next++};mock_gl::programs.insert(p.Program);glUseProgram(p.Program);return p;}
 bool IsValid()const{return Program!=0;}static void Free(GlProgram& p){glUseProgram(0);mock_gl::programs.erase(p.Program);p={};}};
struct ovrGpuState {enum {BLEND_DISABLE};int blendEnable=0;bool depthEnable=true,depthMaskEnable=true,cullEnable=true;
 GLenum depthFunc=GL_LEQUAL,frontFace=GL_CCW;};
struct GraphicsCommand {GlProgram Program;struct Uniform {void* Data=nullptr;};Uniform UniformData[8];ovrGpuState GpuState;};
struct VertexAttribs {std::vector<OVR::Vector3f> position;};
struct GlGeometry {GLuint vertexArrayObject=0,vertexBuffer=0,indexBuffer=0;GLenum primitiveType=GL_TRIANGLES,indexType=GL_UNSIGNED_SHORT;
 int vertexCount=0,indexCount=0;OVR::Bounds3f localBounds;
 static constexpr int kMaxGeometryVertices=65536;static constexpr GLenum kPrimitiveTypePoints=GL_POINTS,kPrimitiveTypeTriangles=GL_TRIANGLES;
 struct TransformScope {TransformScope(const OVR::Matrix4f&,bool){}};
 void Create(const VertexAttribs& v,const std::vector<TriangleIndex>& i){
  glGenVertexArrays(1,&vertexArrayObject);glGenBuffers(1,&vertexBuffer);glGenBuffers(1,&indexBuffer);
  glBindVertexArray(vertexArrayObject);glBindBuffer(GL_ARRAY_BUFFER,vertexBuffer);glBufferData(GL_ARRAY_BUFFER,v.position.size()*sizeof(OVR::Vector3f),v.position.data(),GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,indexBuffer);glBufferData(GL_ELEMENT_ARRAY_BUFFER,i.size()*sizeof(TriangleIndex),i.data(),GL_STATIC_DRAW);
  vertexCount=static_cast<int>(v.position.size());indexCount=static_cast<int>(i.size());}
 void Free(){glDeleteVertexArrays(1,&vertexArrayObject);glDeleteBuffers(1,&vertexBuffer);glDeleteBuffers(1,&indexBuffer);vertexArrayObject=vertexBuffer=indexBuffer=0;vertexCount=indexCount=0;}};
struct ovrSurfaceDef {std::string surfaceName;GlGeometry geo;GraphicsCommand graphicsCommand;};
struct ovrDrawSurface {OVR::Matrix4f modelMatrix;const ovrSurfaceDef* surface;
 ovrDrawSurface(const OVR::Matrix4f& m,const ovrSurfaceDef* s):modelMatrix(m),surface(s){}};
}
