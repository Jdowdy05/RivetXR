#include "arm_renderer.h"
#include "franka_meshes.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace quest_newton;
using DoubleMatrix = std::array<double, 16>;
constexpr double kPositionLimit = 0.001;
constexpr double kOrientationLimitDegrees = 0.1;
constexpr std::array<std::string_view, 12> kExpectedNames{
    "panda_link0", "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5",
    "panda_link6", "panda_link7", "panda_link8", "panda_hand", "panda_leftfinger", "panda_rightfinger"};
constexpr std::array<int, 12> kExpectedAssets{0, 1, 2, 3, 4, 5, 6, 7, -1, 8, 9, 9};

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// QVPF v1, little endian, no padding/trailing bytes:
// magic[4], uint32 version=1, uint32 bodies=12, uint32 visuals=11,
// mesh_manifest_sha256[64] (ASCII hex, no NUL), 84 IEEE float32 body values
// in px,py,pz,qx,qy,qz,qw order, then 12*16 IEEE float64 expected matrices.
// Expected matrices are row-major robot_base-from-visual (column vectors).
class FixtureReader {
  public:
    explicit FixtureReader(const char* path) {
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        Require(input.good(), "cannot open parity fixture");
        constexpr std::size_t size = 80 + 84 * 4 + 12 * 16 * 8;
        Require(input.tellg() == static_cast<std::streamoff>(size), "wrong parity fixture size");
        bytes_.resize(size);
        input.seekg(0);
        input.read(reinterpret_cast<char*>(bytes_.data()), static_cast<std::streamsize>(size));
        Require(input.good(), "cannot read parity fixture");
    }
    std::string Text(std::size_t count) {
        Require(offset_ + count <= bytes_.size(), "truncated parity fixture");
        const std::string result(reinterpret_cast<const char*>(bytes_.data() + offset_), count);
        offset_ += count;
        return result;
    }
    template<class Integer> Integer Unsigned() {
        Require(offset_ + sizeof(Integer) <= bytes_.size(), "truncated parity fixture");
        Integer value = 0;
        for (std::size_t i = 0; i < sizeof(Integer); ++i) value |= static_cast<Integer>(bytes_[offset_++]) << (8 * i);
        return value;
    }
    float Float() { return std::bit_cast<float>(Unsigned<std::uint32_t>()); }
    double Double() { return std::bit_cast<double>(Unsigned<std::uint64_t>()); }
  private:
    std::vector<unsigned char> bytes_;
    std::size_t offset_ = 0;
};

std::array<double, 9> Rotation(const DoubleMatrix& matrix) {
    std::array<double, 9> rotation{};
    for (std::size_t column = 0; column < 3; ++column) {
        const double norm = std::hypot(matrix[column], matrix[4 + column], matrix[8 + column]);
        Require(std::isfinite(norm) && norm > 1e-12, "degenerate visual basis");
        for (std::size_t row = 0; row < 3; ++row) rotation[row * 3 + column] = matrix[row * 4 + column] / norm;
    }
    return rotation;
}
double OrientationError(const DoubleMatrix& actual, const DoubleMatrix& expected) {
    const auto a = Rotation(actual);
    const auto b = Rotation(expected);
    // Trace(R_expected^T * R_actual) after removing positive column scales.
    double trace = 0;
    for (std::size_t i = 0; i < 9; ++i) trace += a[i] * b[i];
    return std::acos(std::clamp((trace - 1.0) * 0.5, -1.0, 1.0)) * 180.0 / std::numbers::pi;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "usage: arm_visual_parity_test <visual_parity.bin>");
        FixtureReader fixture(argv[1]);
        Require(fixture.Text(4) == "QVPF", "wrong parity fixture magic");
        Require(fixture.Unsigned<std::uint32_t>() == 1, "unsupported parity fixture version");
        Require(fixture.Unsigned<std::uint32_t>() == 12, "wrong body count");
        Require(fixture.Unsigned<std::uint32_t>() == 11, "wrong visual count");
        Require(fixture.Text(64) == generated_meshes::kManifestSha256, "compiled mesh manifest differs from fixture");
        Require(generated_meshes::kMeshBodies.size() == 12 && generated_meshes::kVisualBodyCount == 11,
                "wrong compiled body/visual counts");
        BodyValues bodies{};
        for (auto& value : bodies) value = fixture.Float();
        std::array<DoubleMatrix, 12> expected{};
        for (auto& matrix : expected) {
            for (auto& value : matrix) { value = fixture.Double(); Require(std::isfinite(value), "nonfinite expected matrix"); }
            Require(matrix[12] == 0 && matrix[13] == 0 && matrix[14] == 0 && matrix[15] == 1, "invalid homogeneous expected matrix");
        }
        ArmScene scene;
        Require(scene.Update(bodies, Mat4::Identity()), "renderer rejected real Newton body data");
        double max_position = 0;
        double max_orientation = 0;
        std::array<double, 12> positions{}, orientations{};
        std::size_t visuals = 0;
        for (std::size_t i = 0; i < 12; ++i) {
            const auto& binding = generated_meshes::kMeshBodies[i];
            Require(binding.name != nullptr && binding.name == kExpectedNames[i] && ArmBodyNames()[i] == kExpectedNames[i],
                    "generated or renderer body order mismatch");
            Require(binding.asset_index == kExpectedAssets[i], "wrong body-to-asset mapping");
            if (binding.asset_index >= 0) ++visuals;
            Mat4 body_from_visual;
            Require(MakeVisualTransform({binding.position, binding.rotation}, binding.scale, body_from_visual),
                    "renderer rejected generated visual transform");
            const Mat4 rendered = Multiply(scene.Models()[i], body_from_visual);
            DoubleMatrix actual{};
            for (std::size_t j = 0; j < 16; ++j) {
                actual[j] = rendered.m[j];
                Require(std::isfinite(actual[j]), "nonfinite renderer output");
            }
            // Scale mistakes cannot hide behind the normalized orientation test.
            for (std::size_t column = 0; column < 3; ++column) {
                const double a = std::hypot(actual[column], actual[4 + column], actual[8 + column]);
                const double b = std::hypot(expected[i][column], expected[i][4 + column], expected[i][8 + column]);
                Require(std::abs(a - b) < 1e-5, "renderer visual scale mismatch");
            }
            positions[i] = std::hypot(actual[3] - expected[i][3], actual[7] - expected[i][7], actual[11] - expected[i][11]);
            orientations[i] = OrientationError(actual, expected[i]);
            max_position = std::max(max_position, positions[i]);
            max_orientation = std::max(max_orientation, orientations[i]);
        }
        Require(visuals == 11, "not all eleven visuals are represented");
        const bool passed = max_position < kPositionLimit && max_orientation < kOrientationLimitDegrees;
        std::printf("{\"passed\":%s,\"body_count\":12,\"visual_count\":11,\"max_origin_error_m\":%.17g,"
                    "\"max_orientation_error_degrees\":%.17g,\"origin_threshold_m\":0.001,"
                    "\"orientation_threshold_degrees\":0.1,\"mesh_manifest_sha256\":\"%s\",\"bodies\":[",
                    passed ? "true" : "false", max_position, max_orientation, generated_meshes::kManifestSha256);
        for (std::size_t i = 0; i < 12; ++i) {
            std::printf("%s{\"name\":\"%s\",\"origin_error_m\":%.17g,\"orientation_error_degrees\":%.17g}",
                        i == 0 ? "" : ",", generated_meshes::kMeshBodies[i].name, positions[i], orientations[i]);
        }
        std::puts("]}");
        return passed ? 0 : 1;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Visual parity verification failed: %s\n", error.what());
        return 1;
    }
}
