#pragma once
#include "quest_newton/franka_kinematics.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace quest_newton {
inline constexpr std::size_t kRemoteSceneHeaderBytes=128,kRemoteScenePointBytes=16;
inline constexpr std::size_t kRemoteSceneMaxPoints=50000;
inline constexpr std::size_t kRemoteSurfaceHeaderBytes=160,kRemoteSurfaceViewBytes=160,kRemoteSurfaceVertexBytes=24;
inline constexpr std::size_t kRemoteSurfaceMaxVertices=16384,kRemoteSurfaceMaxIndices=98304,kRemoteSurfaceMaxAtlasPixels=384000;
inline constexpr std::uint32_t kRemoteSurfaceMaxAtlasDimension=2048;
inline constexpr std::size_t kRemoteSceneLegacyMaxPacketBytes=973984;
inline constexpr std::size_t kRemoteRgbHeaderBytes=224,kRemoteRgbVertexBytes=44,kRemoteRgbObservationBytes=80,kRemoteRgbViewBytes=208;
inline constexpr std::size_t kRemoteRgbMaxVertices=32768,kRemoteRgbMaxIndices=196608,kRemoteRgbMaxPixels=768000,kRemoteRgbMaxViews=10;
inline constexpr std::size_t kRemoteSceneMaxPacketBytes=4140032;
inline constexpr std::size_t kRemoteSceneMaxTransportBytes=4*1024*1024;
inline constexpr std::uint32_t kRemoteSceneTruncated=1,kRemoteScenePartial=2,kRemoteSceneRecorded=4,kRemoteSceneSynthetic=8;
using RemoteSceneTime=std::chrono::steady_clock::time_point;
enum class RemoteSceneRepresentation : std::uint32_t { Points=0,Prepared=1,Unprepared=2 };
struct RemoteSceneIdentity {
    std::uint64_t source_id=0,world_epoch=0,calibration_id=0;
    RemoteSceneRepresentation representation=RemoteSceneRepresentation::Points;
    std::uint64_t map_generation=0;
    bool operator==(const RemoteSceneIdentity&) const=default;
};
struct RemoteScenePoint {
    kinematics::Vec3 position{};
    std::uint8_t grayscale=0,support=0;
    std::uint16_t radius_mm=0;
};
struct RemoteSceneVertex {
    std::array<float,3> position{},uvq{};
};
static_assert(sizeof(RemoteSceneVertex)==kRemoteSurfaceVertexBytes);
struct RemoteSceneRgbVertex {
    std::array<float,3> position{},uvq{};
    std::array<float,4> tile_bounds{};
    float texture_valid=0;
};
static_assert(sizeof(RemoteSceneRgbVertex)==kRemoteRgbVertexBytes);
struct RemoteSceneObservation {
    std::uint32_t camera_id=0,flags=0;
    std::uint64_t observation_id=0,depth_capture_ns=0,image_capture_ns=0;
    std::uint32_t vertex_start=0,vertex_count=0,index_start=0,index_count=0;
    std::uint32_t tile_x=0,tile_y=0,tile_width=0,tile_height=0,image_format=0;
    bool operator==(const RemoteSceneObservation&) const=default;
};
inline std::uint64_t RemoteObservationCaptureNs(const RemoteSceneObservation& observation){
    return (observation.flags&2U)?std::min(observation.depth_capture_ns,observation.image_capture_ns):observation.depth_capture_ns;
}
struct RemoteSceneFrame {
    std::uint64_t source_id=0,world_epoch=0,sequence=0,calibration_id=0;
    // These three timestamps are in the producer clock, never headset time.
    std::uint64_t capture_start_ns=0,capture_end_ns=0,produced_ns=0;
    std::uint32_t expected_camera_mask=0,contributing_mask=0,flags=0,max_capture_skew_ms=0;
    float voxel_size_m=0;
    // map_from_OpenXR_view: observer-local +X right, +Y up, -Z forward.
    kinematics::Pose suggested_observer;
    std::vector<RemoteScenePoint> points;
    RemoteSceneRepresentation representation=RemoteSceneRepresentation::Points;
    std::vector<RemoteSceneVertex> vertices;
    std::vector<std::uint16_t> indices;
    std::vector<std::uint8_t> atlas;
    std::uint32_t atlas_width=0,atlas_height=0;
    std::uint32_t wire_version=1,atlas_channels=1;
    std::uint64_t map_generation=0,oldest_observation_ns=0;
    std::uint32_t current_view_count=0,retained_view_count=0,retention_ms=0,map_flags=0;
    std::vector<RemoteSceneRgbVertex> rgb_vertices;
    std::vector<RemoteSceneObservation> observations;
    bool HasGeometry() const{return representation==RemoteSceneRepresentation::Points?!points.empty():!indices.empty();}
    RemoteSceneIdentity Identity() const{return {source_id,world_epoch,calibration_id,representation,map_generation};}
};
// Strict RSCN v1/v2 parser. Unprepared v2 images are converted on the caller's
// receive worker, retaining representation identity. Failure leaves output unchanged.
bool DecodeRemoteScene(std::span<const std::byte> packet,RemoteSceneFrame& output,std::string& error);
struct RemoteSceneSnapshot {
    std::shared_ptr<const RemoteSceneFrame> frame;
    RemoteSceneTime received_at{};
    // Advances on each accepted identity transition, including A->B->A between
    // reader samples. Bind inspection alignment to this and stream_generation.
    std::uint64_t stream_generation=0,identity_revision=0,publication=0;
};
double RemoteSceneAgeMs(const RemoteSceneSnapshot& snapshot,RemoteSceneTime now);
bool RemoteSceneFresh(const RemoteSceneSnapshot& snapshot,RemoteSceneTime now,
                      std::chrono::milliseconds max_age=std::chrono::milliseconds(500));
class RemoteSceneMailbox {
public:
    RemoteSceneMailbox();
    ~RemoteSceneMailbox();
    RemoteSceneMailbox(const RemoteSceneMailbox&)=delete;
    RemoteSceneMailbox& operator=(const RemoteSceneMailbox&)=delete;
    // Retire previous frames/producers. Pass the returned token to Publish.
    // At most 64 distinct identities are retained per generation to reject
    // replayed sequences across identity changes; Reset starts a new history.
    std::uint64_t Reset();
    bool Publish(std::span<const std::byte> packet,RemoteSceneTime received_at,
                 std::uint64_t stream_generation,std::string& error);
    RemoteSceneSnapshot Read() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace quest_newton
