#pragma once
#include "remote_scene.h"

namespace quest_newton {
// Internal payload decoder. Common RSCN header/CRC must already be validated.
// Performs all count/calibration/length checks before allocating geometry.
bool DecodeRemoteSurface(std::span<const std::byte> packet,RemoteSceneFrame& frame,std::string& error);
}
