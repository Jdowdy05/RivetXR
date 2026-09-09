#pragma once
// Host-only state/failure fixture. The real NDK compilation separately checks
// the production GL/SDK ABI; these functions do not emulate rasterization.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <tuple>
#include <vector>
using GLenum=unsigned;using GLuint=unsigned;using GLint=int;using GLsizei=int;
using GLboolean=unsigned char;using GLfloat=float;using GLsizeiptr=std::ptrdiff_t;using GLintptr=std::ptrdiff_t;
inline constexpr GLenum GL_FALSE=0,GL_TRUE=1,GL_NO_ERROR=0,GL_OUT_OF_MEMORY=0x505;
inline constexpr GLenum GL_VERTEX_ARRAY_BINDING=1,GL_ARRAY_BUFFER_BINDING=2,GL_ELEMENT_ARRAY_BUFFER_BINDING=3,
 GL_CURRENT_PROGRAM=4,GL_ACTIVE_TEXTURE=5,GL_TEXTURE_BINDING_2D=6,GL_PIXEL_UNPACK_BUFFER_BINDING=7,
 GL_UNPACK_ALIGNMENT=8,GL_UNPACK_ROW_LENGTH=9,GL_UNPACK_SKIP_ROWS=10,GL_UNPACK_SKIP_PIXELS=11,
 GL_MAX_TEXTURE_SIZE=12,GL_CULL_FACE_MODE=13,GL_ALIASED_POINT_SIZE_RANGE=14;
inline constexpr GLenum GL_ARRAY_BUFFER=20,GL_ELEMENT_ARRAY_BUFFER=21,GL_PIXEL_UNPACK_BUFFER=22,
 GL_TEXTURE_2D=23,GL_TEXTURE0=100,GL_FLOAT=30,GL_UNSIGNED_BYTE=31,GL_UNSIGNED_SHORT=32,
 GL_DYNAMIC_DRAW=33,GL_STATIC_DRAW=34,GL_R8=35,GL_RED=36,GL_RGB8=43,GL_RGB=44,GL_TEXTURE_MIN_FILTER=37,
 GL_TEXTURE_MAG_FILTER=38,GL_TEXTURE_WRAP_S=39,GL_TEXTURE_WRAP_T=40,GL_LINEAR=41,GL_CLAMP_TO_EDGE=42,
 GL_POINTS=0,GL_TRIANGLES=4,GL_LEQUAL=0x203,GL_CCW=0x901,GL_BACK=0x405;
namespace mock_gl {
inline GLuint next=1;inline GLenum error=0;inline bool fail_texture=false,fail_buffer=false;
inline unsigned calls=0;
inline std::map<GLenum,GLint> state{{GL_VERTEX_ARRAY_BINDING,900},{GL_ARRAY_BUFFER_BINDING,901},
 {GL_CURRENT_PROGRAM,903},{GL_ACTIVE_TEXTURE,GL_TEXTURE0+3},{GL_PIXEL_UNPACK_BUFFER_BINDING,906},
 {GL_UNPACK_ALIGNMENT,8},{GL_UNPACK_ROW_LENGTH,17},{GL_UNPACK_SKIP_ROWS,5},{GL_UNPACK_SKIP_PIXELS,6},
 {GL_MAX_TEXTURE_SIZE,4096},{GL_CULL_FACE_MODE,GL_BACK}};
inline std::map<GLuint,GLuint> elements{{900,902}};
inline std::map<GLenum,GLuint> textures{{GL_TEXTURE0,904},{GL_TEXTURE0+3,905}};
inline std::set<GLuint> vaos,buffers,texture_ids,programs;
inline std::map<GLuint,std::vector<std::byte>> buffer_data;
inline std::map<GLuint,std::vector<std::uint8_t>> texture_data;
inline std::map<std::pair<GLuint,GLuint>,std::tuple<GLint,GLenum,GLsizei,std::uintptr_t>> attributes;
inline GLuint Bound(GLenum target){return target==GL_ARRAY_BUFFER?state[GL_ARRAY_BUFFER_BINDING]:
 target==GL_ELEMENT_ARRAY_BUFFER?elements[state[GL_VERTEX_ARRAY_BINDING]]:state[GL_PIXEL_UNPACK_BUFFER_BINDING];}
inline auto State(){return std::tuple{state,elements[static_cast<GLuint>(state[GL_VERTEX_ARRAY_BINDING])],textures};}
}
inline void glGetIntegerv(GLenum p,GLint* v){++mock_gl::calls;*v=p==GL_ELEMENT_ARRAY_BUFFER_BINDING?
 mock_gl::elements[mock_gl::state[GL_VERTEX_ARRAY_BINDING]]:p==GL_TEXTURE_BINDING_2D?
 mock_gl::textures[mock_gl::state[GL_ACTIVE_TEXTURE]]:mock_gl::state[p];}
inline void glGetFloatv(GLenum,GLfloat* v){++mock_gl::calls;v[0]=1;v[1]=64;}
inline GLenum glGetError(){++mock_gl::calls;auto e=mock_gl::error;mock_gl::error=0;return e;}
inline void glBindVertexArray(GLuint v){++mock_gl::calls;mock_gl::state[GL_VERTEX_ARRAY_BINDING]=v;}
inline void glBindBuffer(GLenum t,GLuint v){++mock_gl::calls;if(t==GL_ELEMENT_ARRAY_BUFFER)mock_gl::elements[mock_gl::state[GL_VERTEX_ARRAY_BINDING]]=v;
 else mock_gl::state[t==GL_ARRAY_BUFFER?GL_ARRAY_BUFFER_BINDING:GL_PIXEL_UNPACK_BUFFER_BINDING]=v;}
inline void glActiveTexture(GLenum u){++mock_gl::calls;mock_gl::state[GL_ACTIVE_TEXTURE]=static_cast<GLint>(u);}
inline void glBindTexture(GLenum,GLuint v){++mock_gl::calls;mock_gl::textures[mock_gl::state[GL_ACTIVE_TEXTURE]]=v;}
inline void glUseProgram(GLuint p){++mock_gl::calls;mock_gl::state[GL_CURRENT_PROGRAM]=p;}
inline void glPixelStorei(GLenum p,GLint v){++mock_gl::calls;mock_gl::state[p]=v;}
inline void glGenVertexArrays(GLsizei n,GLuint* v){while(n--){++mock_gl::calls;*v=mock_gl::next++;mock_gl::vaos.insert(*v++);}}
inline void glGenBuffers(GLsizei n,GLuint* v){while(n--){++mock_gl::calls;*v=mock_gl::next++;mock_gl::buffers.insert(*v++);}}
inline void glGenTextures(GLsizei n,GLuint* v){while(n--){++mock_gl::calls;*v=mock_gl::next++;mock_gl::texture_ids.insert(*v++);}}
inline void glDeleteVertexArrays(GLsizei n,const GLuint* v){while(n--){++mock_gl::calls;mock_gl::vaos.erase(*v);mock_gl::elements.erase(*v++);}}
inline void glDeleteBuffers(GLsizei n,const GLuint* v){while(n--){++mock_gl::calls;mock_gl::buffers.erase(*v);mock_gl::buffer_data.erase(*v++);}}
inline void glDeleteTextures(GLsizei n,const GLuint* v){while(n--){++mock_gl::calls;mock_gl::texture_ids.erase(*v);mock_gl::texture_data.erase(*v++);}}
inline void glBufferData(GLenum t,GLsizeiptr n,const void* p,GLenum){++mock_gl::calls;auto& b=mock_gl::buffer_data[mock_gl::Bound(t)];b.resize(n);if(p)std::memcpy(b.data(),p,n);}
inline void glBufferSubData(GLenum t,GLintptr o,GLsizeiptr n,const void* p){++mock_gl::calls;if(mock_gl::fail_buffer){mock_gl::error=GL_OUT_OF_MEMORY;return;}
 auto& b=mock_gl::buffer_data[mock_gl::Bound(t)];if(n)std::memcpy(b.data()+o,p,n);}
inline void glEnableVertexAttribArray(GLuint){++mock_gl::calls;}
inline void glDisableVertexAttribArray(GLuint){++mock_gl::calls;}
inline void glVertexAttribPointer(GLuint a,GLint n,GLenum t,GLboolean,GLsizei s,const void* p){++mock_gl::calls;mock_gl::attributes[{mock_gl::state[GL_VERTEX_ARRAY_BINDING],a}]={n,t,s,reinterpret_cast<std::uintptr_t>(p)};}
inline void glTexParameteri(GLenum,GLenum,GLint){++mock_gl::calls;}
inline void glTexImage2D(GLenum,GLint,GLint,GLsizei w,GLsizei h,GLint,GLenum format,GLenum,const void* p){
 ++mock_gl::calls;if(mock_gl::fail_texture){mock_gl::error=GL_OUT_OF_MEMORY;return;}
 auto& b=mock_gl::texture_data[mock_gl::textures[mock_gl::state[GL_ACTIVE_TEXTURE]]];b.resize(static_cast<std::size_t>(w)*h*(format==GL_RGB?3:1));
 if(p)std::memcpy(b.data(),p,b.size());}
