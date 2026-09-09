#include "menu_pointer_renderer.h"

#include "GUI/VRMenuObject.h"
#include "Render/BeamRenderer.h"
#include "Render/ParticleSystem.h"
#include <GLES3/gl3.h>
#include <array>
#include <cmath>
#include <limits>

namespace quest_newton {
struct MenuPointerRenderer::Impl {
    struct Pointer {
        OVR::Vector3f start{}, end{};
        bool visible=false, pressed=false;
    };
    // Persistent slots avoid v85 SimpleBeamRenderer's growing handle history
    // and the low-level renderers' deferred remove/recycle behavior.
    OVRFW::ovrBeamRenderer beams;
    OVRFW::ovrParticleSystem cursors;
    std::array<OVRFW::ovrBeamRenderer::handle_t,2> beam_handles{};
    std::array<OVRFW::ovrParticleSystem::handle_t,4> cursor_handles{};
    std::array<Pointer,2> pointers{};
    bool ready=false, allocated=false;
};

MenuPointerRenderer::MenuPointerRenderer():impl_(std::make_unique<Impl>()){}
// The owner must destroy this while EGL is current, in SessionEnd. SDK
// destructors release their GL resources and all per-session CPU storage.
MenuPointerRenderer::~MenuPointerRenderer()=default;

bool MenuPointerRenderer::Init(){
    auto& p=*impl_;
    if(p.ready)return true;
    const OVRFW::GlGeometry::TransformScope identity(OVR::Matrix4f::Identity(),false);
    p.beams.Init(2,false);
    auto state=OVRFW::ovrParticleSystem::GetDefaultGpuState();
    state.depthEnable=false;state.depthMaskEnable=false;state.cullEnable=false;
    // The procedural cursor shader emits premultiplied coverage. A black
    // outer disc plus a smaller bright disc stays legible on text and clouds.
    state.blendSrc=GL_ONE;state.blendDst=GL_ONE_MINUS_SRC_ALPHA;
    p.cursors.Init(4,nullptr,state,false);
    p.ready=glGetError()==GL_NO_ERROR;
    return p.ready;
}

void MenuPointerRenderer::Update(std::span<const OVRFW::TinyUI::HitTestDevice> devices){
    auto& p=*impl_;p.pointers={};
    for(const auto& device:devices){
        if(device.deviceNum<0 || device.deviceNum>=2 || !device.hitObject ||
           (device.hitObject->GetFlags() & OVRFW::VRMENUOBJECT_DONT_RENDER))continue;
        const auto& start=device.pointerStart;const auto& end=device.pointerEnd;
        if(!std::isfinite(start.x)||!std::isfinite(start.y)||!std::isfinite(start.z)||
           !std::isfinite(end.x)||!std::isfinite(end.y)||!std::isfinite(end.z)||
           (end-start).LengthSq()<1e-8F)continue;
        p.pointers[static_cast<std::size_t>(device.deviceNum)]={start,end,true,device.clicked};
    }
}

void MenuPointerRenderer::Render(const OVRFW::ovrApplFrameIn& in,OVRFW::ovrRendererOutput& out){
    auto& p=*impl_;
    if(!p.ready || (!p.pointers[0].visible && !p.pointers[1].visible))return;
    constexpr float lifetime=std::numeric_limits<float>::max();
    const OVR::Vector3f zero{0,0,0};
    const OVR::Vector4f invisible{0,0,0,0};
    if(!p.allocated){
        for(std::size_t i=0;i<2;++i){
            p.beam_handles[i]=p.beams.AddBeam(in,.003F,zero,{0,0,-1},invisible);
            for(std::size_t disc=0;disc<2;++disc)
                p.cursor_handles[i*2+disc]=p.cursors.AddParticle(in,zero,0,zero,zero,
                    invisible,OVRFW::ovrEaseFunc::NONE,0,.01F,lifetime,0);
        }
        p.allocated=true;
    }
    const auto eye=out.FrameMatrices.CenterView.Inverted().GetTranslation();
    const auto hidden_start=eye+OVR::Vector3f{.1F,0,-.1F};
    const auto hidden_end=hidden_start+OVR::Vector3f{0,0,-.1F};
    for(std::size_t i=0;i<2;++i){
        const auto& pointer=p.pointers[i];
        const OVR::Vector4f color=pointer.pressed?OVR::Vector4f{.2F,1.F,.45F,1.F}:
            OVR::Vector4f{.35F,.85F,1.F,1.F};
        // SDK beam billboarding normalizes this cross product. A ray directly
        // through the eye has no screen-space width; keep its cursor but avoid
        // a zero-vector normalization, including for inactive transparent slots.
        const bool beam_visible=pointer.visible &&
            (pointer.end-pointer.start).Cross((pointer.start+pointer.end)*.5F-eye).LengthSq()>1e-10F;
        p.beams.UpdateBeam(in,p.beam_handles[i],.003F,
            beam_visible?pointer.start:hidden_start,beam_visible?pointer.end:hidden_end,
            beam_visible?color:invisible);
        const OVR::Vector4f dot=pointer.pressed?color:OVR::Vector4f{1,1,1,1};
        for(std::size_t disc=0;disc<2;++disc){
            p.cursors.UpdateParticle(in,p.cursor_handles[i*2+disc],pointer.end,0,zero,zero,
                pointer.visible?(disc?dot:OVR::Vector4f{0,0,0,1}):invisible,
                OVRFW::ovrEaseFunc::NONE,0,disc?.010F:.018F,lifetime,0);
        }
    }
    p.beams.Frame(in,out.FrameMatrices.CenterView);
    p.beams.Render(out.Surfaces);
    p.cursors.Frame(in,nullptr,out.FrameMatrices.CenterView);
    p.cursors.RenderEyeView(out.FrameMatrices.CenterView,out.FrameMatrices.EyeProjection[0],out.Surfaces);
}
} // namespace quest_newton
