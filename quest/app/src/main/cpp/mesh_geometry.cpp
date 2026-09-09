#include "mesh_geometry.h"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <new>
#include <string_view>
#include <utility>

namespace quest_newton {
namespace {

constexpr std::uint64_t kHeaderBytes = 20;
constexpr std::uint64_t kPositionBytes = 3 * sizeof(float);
constexpr std::uint64_t kIndexBytes = sizeof(std::uint16_t);
constexpr std::uint64_t kMaximumMeshBytes = 12ULL * 1024ULL * 1024ULL;

bool Fail(std::string& error, std::string_view message) {
    error.assign(message);
    return false;
}

std::uint16_t ReadU16(std::span<const std::byte> bytes, std::size_t offset) {
    return static_cast<std::uint16_t>(
        std::to_integer<std::uint16_t>(bytes[offset]) |
        (std::to_integer<std::uint16_t>(bytes[offset + 1]) << 8U));
}

std::uint32_t ReadU32(std::span<const std::byte> bytes, std::size_t offset) {
    std::uint32_t value = 0;
    for (unsigned shift = 0; shift < 32; shift += 8) {
        value |= std::to_integer<std::uint32_t>(bytes[offset + shift / 8]) << shift;
    }
    return value;
}

float ReadFloat(std::span<const std::byte> bytes, std::size_t offset) {
    return std::bit_cast<float>(ReadU32(bytes, offset));
}

} // namespace

bool DecodeQmsh(
    std::span<const std::byte> bytes,
    MeshGeometryData& output,
    std::string& error) {
    if (bytes.size() < kHeaderBytes) {
        return Fail(error, "QMSH header is truncated");
    }
    if (bytes[0] != static_cast<std::byte>('Q') ||
        bytes[1] != static_cast<std::byte>('M') ||
        bytes[2] != static_cast<std::byte>('S') ||
        bytes[3] != static_cast<std::byte>('H')) {
        return Fail(error, "QMSH magic is invalid");
    }
    if (ReadU32(bytes, 4) != 1) {
        return Fail(error, "QMSH version is unsupported");
    }
    if (ReadU32(bytes, 8) != 0) {
        return Fail(error, "QMSH flags are unsupported");
    }

    const std::uint32_t vertex_count = ReadU32(bytes, 12);
    const std::uint32_t index_count = ReadU32(bytes, 16);
    if (vertex_count < 3 || vertex_count > 65535) {
        return Fail(error, "QMSH vertex count must be between 3 and 65535");
    }
    if (index_count < 3) {
        return Fail(error, "QMSH index count must be at least 3");
    }
    if (index_count % 3 != 0) {
        return Fail(error, "QMSH index count must contain complete triangles");
    }

    const std::uint64_t expected_bytes =
        kHeaderBytes +
        static_cast<std::uint64_t>(vertex_count) * kPositionBytes +
        static_cast<std::uint64_t>(index_count) * kIndexBytes;
    if (expected_bytes > kMaximumMeshBytes) {
        return Fail(error, "QMSH mesh exceeds the 12 MiB limit");
    }
    if (static_cast<std::uint64_t>(bytes.size()) < expected_bytes) {
        return Fail(error, "QMSH payload is truncated");
    }
    if (static_cast<std::uint64_t>(bytes.size()) > expected_bytes) {
        return Fail(error, "QMSH payload has trailing bytes");
    }

    MeshGeometryData candidate;
    try {
        candidate.positions.resize(vertex_count);
        candidate.indices.resize(index_count);
    } catch (const std::bad_alloc&) {
        return Fail(error, "QMSH geometry allocation failed");
    }

    std::size_t offset = static_cast<std::size_t>(kHeaderBytes);
    for (auto& position : candidate.positions) {
        for (float& coordinate : position) {
            coordinate = ReadFloat(bytes, offset);
            if (!std::isfinite(coordinate)) {
                return Fail(error, "QMSH position contains a non-finite coordinate");
            }
            offset += static_cast<std::size_t>(sizeof(float));
        }
    }
    for (std::uint16_t& index : candidate.indices) {
        index = ReadU16(bytes, offset);
        if (index >= vertex_count) {
            return Fail(error, "QMSH index is outside the vertex range");
        }
        offset += static_cast<std::size_t>(sizeof(std::uint16_t));
    }

    output = std::move(candidate);
    error.clear();
    return true;
}

} // namespace quest_newton
