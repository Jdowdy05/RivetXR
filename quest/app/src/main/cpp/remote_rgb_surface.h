#pragma once
#include "remote_scene.h"
namespace quest_newton {
// Internal v3 payload path; common framing/CRC are validated by DecodeRemoteScene.
bool DecodeRemoteRgbSurface(std::span<const std::byte> packet,RemoteSceneFrame& frame,std::string& error);
}
