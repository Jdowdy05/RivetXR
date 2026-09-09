#include "mesh_geometry.h"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using quest_newton::DecodeQmsh;
using quest_newton::MeshGeometryData;

void Check(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

void AppendU16(std::vector<std::byte>& bytes, std::uint16_t value) {
    bytes.push_back(static_cast<std::byte>(value & 0xffU));
    bytes.push_back(static_cast<std::byte>((value >> 8U) & 0xffU));
}

void AppendU32(std::vector<std::byte>& bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) {
        bytes.push_back(static_cast<std::byte>((value >> shift) & 0xffU));
    }
}

void AppendFloat(std::vector<std::byte>& bytes, float value) {
    AppendU32(bytes, std::bit_cast<std::uint32_t>(value));
}

void StoreU32(std::vector<std::byte>& bytes, std::size_t offset, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) {
        bytes[offset + shift / 8] = static_cast<std::byte>((value >> shift) & 0xffU);
    }
}

void StoreFloat(std::vector<std::byte>& bytes, std::size_t offset, float value) {
    StoreU32(bytes, offset, std::bit_cast<std::uint32_t>(value));
}

std::vector<std::byte> MakeMesh(
    const std::vector<std::array<float, 3>>& positions,
    const std::vector<std::uint16_t>& indices) {
    std::vector<std::byte> bytes;
    bytes.reserve(20 + positions.size() * 12 + indices.size() * 2);
    bytes.insert(bytes.end(), {
        static_cast<std::byte>('Q'), static_cast<std::byte>('M'),
        static_cast<std::byte>('S'), static_cast<std::byte>('H')});
    AppendU32(bytes, 1);
    AppendU32(bytes, 0);
    AppendU32(bytes, static_cast<std::uint32_t>(positions.size()));
    AppendU32(bytes, static_cast<std::uint32_t>(indices.size()));
    for (const auto& position : positions) {
        for (float coordinate : position) {
            AppendFloat(bytes, coordinate);
        }
    }
    for (std::uint16_t index : indices) {
        AppendU16(bytes, index);
    }
    return bytes;
}

std::vector<std::byte> TriangleBytes() {
    return MakeMesh(
        {{{1.25F, -2.5F, 3.75F}, {0.0F, 4.0F, -8.0F}, {-1.0F, 0.5F, 2.0F}}},
        {2, 0, 1});
}

void ExpectRejected(
    const std::vector<std::byte>& bytes,
    std::string_view case_name,
    MeshGeometryData* destination = nullptr) {
    MeshGeometryData local;
    MeshGeometryData& output = destination == nullptr ? local : *destination;
    std::string error = "stale error";
    Check(!DecodeQmsh(bytes, output, error), case_name);
    Check(!error.empty(), "rejection must explain the error");
}

void DecodesHandDerivedTriangle() {
    const auto bytes = TriangleBytes();
    MeshGeometryData output;
    std::string error = "stale error";

    Check(DecodeQmsh(bytes, output, error), error);
    Check(error.empty(), "success must clear a stale error");
    Check(output.positions ==
              (std::vector<std::array<float, 3>>{
                  {1.25F, -2.5F, 3.75F},
                  {0.0F, 4.0F, -8.0F},
                  {-1.0F, 0.5F, 2.0F}}),
          "decoded positions differ from the hand-derived fixture");
    Check(output.indices == (std::vector<std::uint16_t>{2, 0, 1}),
          "decoded indices differ from the hand-derived fixture");
}

void RejectsHeaderSchemaErrors() {
    auto wrong_magic = TriangleBytes();
    wrong_magic[0] = static_cast<std::byte>('X');
    ExpectRejected(wrong_magic, "wrong magic must fail");

    auto wrong_version = TriangleBytes();
    StoreU32(wrong_version, 4, 2);
    ExpectRejected(wrong_version, "wrong version must fail");

    auto flags = TriangleBytes();
    StoreU32(flags, 8, 1);
    ExpectRejected(flags, "unsupported flags must fail");

    auto truncated_header = TriangleBytes();
    truncated_header.resize(19);
    ExpectRejected(truncated_header, "truncated header must fail");
}

void RejectsSizeMismatch() {
    auto truncated_payload = TriangleBytes();
    truncated_payload.pop_back();
    ExpectRejected(truncated_payload, "truncated payload must fail");

    auto trailing_byte = TriangleBytes();
    trailing_byte.push_back(std::byte{0});
    ExpectRejected(trailing_byte, "trailing byte must fail");
}

void RejectsInvalidCountsBeforeAllocation() {
    auto zero_vertices = TriangleBytes();
    StoreU32(zero_vertices, 12, 0);
    ExpectRejected(zero_vertices, "zero vertices must fail");

    auto too_few_vertices = TriangleBytes();
    StoreU32(too_few_vertices, 12, 2);
    ExpectRejected(too_few_vertices, "fewer than three vertices must fail");

    auto too_many_vertices = TriangleBytes();
    StoreU32(too_many_vertices, 12, 65536);
    ExpectRejected(too_many_vertices, "more than 65535 vertices must fail");

    auto zero_indices = TriangleBytes();
    StoreU32(zero_indices, 16, 0);
    ExpectRejected(zero_indices, "zero indices must fail");

    auto too_few_indices = TriangleBytes();
    StoreU32(too_few_indices, 16, 2);
    ExpectRejected(too_few_indices, "fewer than three indices must fail");

    auto incomplete_triangle = TriangleBytes();
    StoreU32(incomplete_triangle, 16, 4);
    ExpectRejected(incomplete_triangle, "non-triangle index count must fail");

    auto enormous_indices = TriangleBytes();
    StoreU32(enormous_indices, 16, std::numeric_limits<std::uint32_t>::max());
    ExpectRejected(enormous_indices, "oversized index count must fail");
}

void RejectsEveryNonFiniteCoordinate() {
    constexpr std::array<float, 3> non_finite{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()};
    for (std::size_t coordinate = 0; coordinate < 9; ++coordinate) {
        for (float value : non_finite) {
            auto bytes = TriangleBytes();
            StoreFloat(bytes, 20 + coordinate * sizeof(float), value);
            ExpectRejected(bytes, "every non-finite coordinate must fail");
        }
    }
}

void RejectsOutOfRangeIndex() {
    auto bytes = TriangleBytes();
    const std::size_t indices_offset = 20 + 3 * 3 * sizeof(float);
    bytes[indices_offset] = std::byte{3};
    bytes[indices_offset + 1] = std::byte{0};
    ExpectRejected(bytes, "index equal to vertex count must fail");
}

void FailurePreservesPreviousGeometry() {
    MeshGeometryData output{
        {{{9.0F, 8.0F, 7.0F}, {6.0F, 5.0F, 4.0F}, {3.0F, 2.0F, 1.0F}}},
        {0, 2, 1}};
    const MeshGeometryData previous = output;
    auto bytes = TriangleBytes();
    bytes.back() = std::byte{3};
    ExpectRejected(bytes, "invalid mesh must fail", &output);
    Check(output.positions == previous.positions && output.indices == previous.indices,
          "failed decode must preserve previous geometry");
}

void DecodesUnalignedInput() {
    const auto aligned = TriangleBytes();
    std::vector<std::byte> storage;
    storage.reserve(aligned.size() + 1);
    storage.push_back(std::byte{0x5a});
    storage.insert(storage.end(), aligned.begin(), aligned.end());

    MeshGeometryData output;
    std::string error;
    Check(DecodeQmsh(std::span<const std::byte>(storage).subspan(1), output, error), error);
    Check(output.positions.front() == (std::array<float, 3>{1.25F, -2.5F, 3.75F}),
          "unaligned decode corrupted a position");
    Check(output.indices == (std::vector<std::uint16_t>{2, 0, 1}),
          "unaligned decode corrupted indices");
}

void AcceptsMaximumVertexCountAndChecksIndexBoundary() {
    std::vector<std::array<float, 3>> positions(65535, {0.0F, 0.0F, 0.0F});
    positions.front() = {1.0F, 2.0F, 3.0F};
    positions.back() = {4.0F, 5.0F, 6.0F};
    auto bytes = MakeMesh(positions, {0, 65534, 1});

    MeshGeometryData output;
    std::string error;
    Check(DecodeQmsh(bytes, output, error), error);
    Check(output.positions.size() == 65535, "maximum valid vertex count changed");
    Check(output.positions.back() == (std::array<float, 3>{4.0F, 5.0F, 6.0F}),
          "last maximum-count vertex was not decoded");
    Check(output.indices == (std::vector<std::uint16_t>{0, 65534, 1}),
          "largest valid index was not decoded");

    const std::size_t indices_offset = 20 + positions.size() * 3 * sizeof(float);
    bytes[indices_offset + 2] = std::byte{0xff};
    bytes[indices_offset + 3] = std::byte{0xff};
    const MeshGeometryData previous = output;
    ExpectRejected(bytes, "uint16 maximum is invalid for 65535 vertices", &output);
    Check(output.positions == previous.positions && output.indices == previous.indices,
          "boundary rejection must preserve the last valid geometry");
}
} // namespace

int main() {
    try {
        DecodesHandDerivedTriangle();
        RejectsHeaderSchemaErrors();
        RejectsSizeMismatch();
        RejectsInvalidCountsBeforeAllocation();
        RejectsEveryNonFiniteCoordinate();
        RejectsOutOfRangeIndex();
        FailurePreservesPreviousGeometry();
        DecodesUnalignedInput();
        AcceptsMaximumVertexCountAndChecksIndexBoundary();
        std::puts("QMSH mesh geometry tests passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
