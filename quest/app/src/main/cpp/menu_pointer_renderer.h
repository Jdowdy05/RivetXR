#pragma once

#include <memory>
#include <span>
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-pedantic"
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "Input/TinyUI.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace quest_newton {

// XR-thread/session-owned visuals for the menu's already-resolved click rays.
// This class never performs hit testing or generates input actions.
class MenuPointerRenderer {
public:
    MenuPointerRenderer();
    ~MenuPointerRenderer();
    bool Init();
    void Update(std::span<const OVRFW::TinyUI::HitTestDevice> devices);
    void Render(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace quest_newton
