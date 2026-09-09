#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace quest_newton {

struct MeshGeometryData {
    std::vector<std::array<float, 3>> positions;
    std::vector<std::uint16_t> indices;
};

bool DecodeQmsh(
    std::span<const std::byte> bytes,
    MeshGeometryData& output,
    std::string& error);

} // namespace quest_newton
