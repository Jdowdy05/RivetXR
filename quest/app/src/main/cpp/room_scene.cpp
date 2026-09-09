#include "room_scene.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <set>
#include <string_view>
#include <utility>

namespace quest_newton {
namespace {
namespace kin = kinematics;
constexpr std::uint32_t kMaxRooms = 16, kMaxMembers = 128, kMaxBoundaryVertices = 256;
constexpr std::size_t kMaxQueryUuids = 50;
constexpr XrDuration kQueryTimeout = 10000000000LL;
constexpr float kHalfThickness = .025F;
constexpr auto kMonitorSweepInterval = std::chrono::milliseconds(500);
constexpr auto kMonitorMaxAge = std::chrono::seconds(1);
constexpr std::size_t kMonitorProbesPerUpdate = 4;
constexpr double kMonitorMaxCornerDistance = .02;
using Clock = std::chrono::steady_clock;
using Uuid = std::array<std::uint8_t, XR_UUID_SIZE_EXT>;
Uuid Key(const XrUuidEXT& uuid) {
    Uuid key{}; std::copy(std::begin(uuid.data), std::end(uuid.data), key.begin()); return key;
}
bool ValidUuid(const XrUuidEXT& uuid) {
    return std::any_of(std::begin(uuid.data), std::end(uuid.data), [](auto b) { return b != 0; });
}
kin::Pose Pose(const XrPosef& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
}
bool SamePose(const kin::Pose& a, const kin::Pose& b) {
    for (std::size_t i = 0; i < 3; ++i)
        if (std::abs(a.position[i] - b.position[i]) > .0001F) return false;
    float dot = 0;
    for (std::size_t i = 0; i < 4; ++i) dot += a.rotation[i] * b.rotation[i];
    return std::abs(dot) > .999999F;
}
bool BoundsValid(const XrRect2Df& r) {
    return std::isfinite(r.offset.x) && std::isfinite(r.offset.y) &&
           std::isfinite(r.extent.width) && std::isfinite(r.extent.height) &&
           std::abs(r.offset.x) <= 100 && std::abs(r.offset.y) <= 100 &&
           r.extent.width >= .01F && r.extent.width <= 40 && r.extent.height >= .01F && r.extent.height <= 40;
}
bool HasLabel(std::string_view labels, std::string_view desired) {
    while (!labels.empty()) {
        const auto end = labels.find_first_of(",\0", 0, 2);
        if (labels.substr(0, end) == desired) return true;
        if (end == std::string_view::npos) break;
        labels.remove_prefix(end + 1);
    }
    return false;
}
bool Contains(const std::vector<XrVector2f>& polygon, float x, float y) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto& a = polygon[j]; const auto& b = polygon[i];
        const double cross = (x - a.x) * (b.y - a.y) - (y - a.y) * (b.x - a.x);
        if (std::abs(cross) < 1e-6 && x >= std::min(a.x, b.x) - 1e-5F &&
            x <= std::max(a.x, b.x) + 1e-5F && y >= std::min(a.y, b.y) - 1e-5F &&
            y <= std::max(a.y, b.y) + 1e-5F) return true;
        if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}
template<class Function>
bool Load(XrInstance instance, const char* name, Function& function) {
    function = nullptr;
    return XR_SUCCEEDED(xrGetInstanceProcAddr(instance, name,
        reinterpret_cast<PFN_xrVoidFunction*>(&function))) && function;
}
} // namespace

struct RoomScene::Impl {
    enum class State { Idle, NeedRooms, Rooms, SelectRoom, Members, Build, Complete, Failed, Invalid, NeedsRefresh };
    enum class QueryKind { Rooms, Floors, Members };
    struct Query {
        XrAsyncRequestIdFB id = 0;
        QueryKind kind = QueryKind::Rooms;
        std::uint32_t limit = 0;
        bool stale = false;
        bool timed_out = false;
        std::vector<XrUuidEXT> expected;
        std::vector<XrSpaceQueryResultFB> results;
        Clock::time_point started{};
    };
    struct Room {
        XrSpace space = XR_NULL_HANDLE;
        XrUuidEXT floor{};
        std::vector<XrUuidEXT> walls;
    };
    struct Anchor {
        XrSpace space = XR_NULL_HANDLE;
        XrUuidEXT uuid{};
        RoomSurfaceKind kind = RoomSurfaceKind::Floor;
        XrRect2Df bounds{};
        std::vector<XrVector2f> boundary;
        XrAsyncRequestIdFB activation = 0;
        bool locating = false;
        bool activation_failed = false;
    };
    struct AnchorMonitor {
        std::array<kin::Vec3, 8> anchor_corners, captured_stage_corners;
        Clock::time_point verified_at{};
    };
    PFN_xrQuerySpacesFB query_spaces = nullptr;
    PFN_xrRetrieveSpaceQueryResultsFB retrieve = nullptr;
    PFN_xrEnumerateSpaceSupportedComponentsFB enumerate_components = nullptr;
    PFN_xrGetSpaceComponentStatusFB get_status = nullptr;
    PFN_xrSetSpaceComponentStatusFB set_status = nullptr;
    PFN_xrGetSpaceRoomLayoutFB get_layout = nullptr;
    PFN_xrGetSpaceContainerFB get_container = nullptr;
    PFN_xrGetSpaceBoundingBox2DFB get_bounds = nullptr;
    PFN_xrGetSpaceBoundary2DFB get_boundary = nullptr;
    PFN_xrGetSpaceSemanticLabelsFB get_labels = nullptr;
    PFN_xrRequestSceneCaptureFB request_capture = nullptr;
    XrSession session = XR_NULL_HANDLE;
    XrSpace capture_stage = XR_NULL_HANDLE;
    XrTime stage_change_time = 0;
    kin::Pose capture_frame;
    bool frame_set = false, supported = false, enabled = false;
    bool requires_explicit_refresh = false;
    bool tracked = false, permission = false, capture_requested = false;
    XrAsyncRequestIdFB capture_id = 0;
    State state = State::Idle;
    std::optional<Query> query;
    std::set<XrSpace> owned;
    std::vector<Room> rooms;
    std::vector<Anchor> floors, surfaces;
    // Parallel to the supported selected-member surfaces, never candidate rooms.
    std::vector<AnchorMonitor> monitors;
    std::size_t monitor_cursor = 0;
    bool monitor_sweep_active = false;
    Clock::time_point next_monitor_sweep{};
    std::set<Uuid> invisible_walls;
    std::vector<XrUuidEXT> member_ids;
    std::size_t next_member = 0, unlocated_rooms = 0;
    std::optional<std::size_t> selected_room;
    RoomEnvironment snapshot;
    std::string status = "Room collisions off";
    std::string failure;
    RoomScene::ClockNow now;
    Clock::time_point locating_started{};

    explicit Impl(RoomScene::ClockNow clock) : now(std::move(clock)) {
        if (!now) now = [] { return Clock::now(); };
        locating_started = now();
    }

    void ReleaseSpaces() {
        // Adopt every copied query handle exactly once, including stale results.
        for (const auto space : owned) xrDestroySpace(space);
        owned.clear(); rooms.clear(); floors.clear(); surfaces.clear(); invisible_walls.clear(); selected_room.reset();
        member_ids.clear(); next_member = 0;
        monitors.clear(); monitor_cursor = 0; monitor_sweep_active = false;
    }
    void RetainAppliedSpaces() {
        std::set<XrSpace> retained;
        for (const auto& surface : surfaces) retained.insert(surface.space);
        for (auto it = owned.begin(); it != owned.end();) {
            if (retained.contains(*it)) ++it;
            else { xrDestroySpace(*it); it = owned.erase(it); }
        }
        rooms.clear(); floors.clear(); invisible_walls.clear(); selected_room.reset();
        member_ids.clear(); next_member = 0;
    }
    void Clear() {
        if (query) { query->stale = true; query->results.clear(); }
        ReleaseSpaces(); frame_set = false; unlocated_rooms = 0;
        snapshot.enabled = false; snapshot.colliders.clear(); ++snapshot.revision;
    }
    void Fail(std::string message) {
        LatchExpiredMonitor();
        if (requires_explicit_refresh) {
            ReleaseSpaces(); state = State::NeedsRefresh; status = failure; return;
        }
        Clear(); state = State::Failed; failure = std::move(message); status = failure;
    }
    void RequireRefresh(std::string message) {
        // Physics still owns the preceding immutable snapshot. Invalidating its
        // health must neither publish Off nor silently move/remove its colliders.
        requires_explicit_refresh = true;
        ReleaseSpaces(); state = State::NeedsRefresh; failure = std::move(message); status = failure;
    }
    bool MonitorFresh(Clock::time_point checked_at) const {
        return !monitors.empty() && monitors.size() == surfaces.size() &&
            std::all_of(monitors.begin(), monitors.end(), [&](const auto& monitor) {
                return checked_at >= monitor.verified_at && checked_at - monitor.verified_at <= kMonitorMaxAge;
            });
    }
    void LatchExpiredMonitor() {
        if (state == State::Complete && !MonitorFresh(now()))
            RequireRefresh("Room verification expired; Refresh room");
    }
    bool Component(XrSpace space, XrSpaceComponentTypeFB component, bool& supported_out) {
        std::array<XrSpaceComponentTypeFB, 32> types{};
        std::uint32_t count = 0;
        if (XR_FAILED(enumerate_components(space, static_cast<std::uint32_t>(types.size()), &count, types.data())) ||
            count > types.size()) return false;
        supported_out = std::find(types.begin(), types.begin() + count, component) != types.begin() + count;
        return true;
    }
    bool EnabledComponent(XrSpace space, XrSpaceComponentTypeFB component) {
        XrSpaceComponentStatusFB value{XR_TYPE_SPACE_COMPONENT_STATUS_FB};
        return XR_SUCCEEDED(get_status(space, component, &value)) && value.enabled && !value.changePending;
    }
    bool StartQuery(QueryKind kind, std::vector<XrUuidEXT> ids = {}) {
        if (query || !session) return false;
        if (ids.size() > kMaxQueryUuids) { Fail("Room query exceeds UUID batch capacity"); return false; }
        XrSpaceStorageLocationFilterInfoFB storage{XR_TYPE_SPACE_STORAGE_LOCATION_FILTER_INFO_FB};
        storage.location = XR_SPACE_STORAGE_LOCATION_LOCAL_FB;
        XrSpaceComponentFilterInfoFB component{XR_TYPE_SPACE_COMPONENT_FILTER_INFO_FB};
        component.next = &storage; component.componentType = XR_SPACE_COMPONENT_TYPE_ROOM_LAYOUT_FB;
        XrSpaceUuidFilterInfoFB uuids{XR_TYPE_SPACE_UUID_FILTER_INFO_FB};
        uuids.next = &storage; uuids.uuidCount = static_cast<std::uint32_t>(ids.size()); uuids.uuids = ids.data();
        XrSpaceQueryInfoFB info{XR_TYPE_SPACE_QUERY_INFO_FB};
        info.queryAction = XR_SPACE_QUERY_ACTION_LOAD_FB;
        // One extra result detects truncation rather than selecting from a
        // silently incomplete room list. UUID queries also reject extra results.
        info.maxResultCount = kind == QueryKind::Rooms ? kMaxRooms + 1 : uuids.uuidCount + 1;
        info.timeout = kQueryTimeout;
        info.filter = kind == QueryKind::Rooms
            ? reinterpret_cast<XrSpaceFilterInfoBaseHeaderFB*>(&component)
            : reinterpret_cast<XrSpaceFilterInfoBaseHeaderFB*>(&uuids);
        Query next; next.kind = kind; next.limit = info.maxResultCount; next.expected = ids; next.started = now();
        if (XR_FAILED(query_spaces(session, reinterpret_cast<XrSpaceQueryInfoBaseHeaderFB*>(&info), &next.id)) || !next.id) {
            Fail("Room query failed; Refresh or Scan room"); return false;
        }
        query = std::move(next); return true;
    }
    bool StartMemberBatch() {
        const auto end = std::min(member_ids.size(), next_member + kMaxQueryUuids);
        std::vector<XrUuidEXT> batch(member_ids.begin() + next_member, member_ids.begin() + end);
        next_member = end;
        if (!StartQuery(QueryKind::Members, std::move(batch))) return false;
        state = State::Members; status = "Loading selected room surfaces"; return true;
    }
    std::string ReadyStatus() const {
        std::string text = "Room ready: " + std::to_string(snapshot.colliders.size()) + " surfaces";
        if (unlocated_rooms) text += "; " + std::to_string(unlocated_rooms) + " other rooms not located";
        return text;
    }
    void Retrieve() {
        if (!query) return;
        XrSpaceQueryResultsFB output{XR_TYPE_SPACE_QUERY_RESULTS_FB};
        if (XR_FAILED(retrieve(session, query->id, &output))) { Fail("Cannot retrieve room query results"); return; }
        if (!output.resultCountOutput) return;
        // The runtime must respect maxResultCount. Never allocate from an
        // unbounded output count; handles not retrieved remain runtime-owned.
        if (output.resultCountOutput > query->limit) { Fail("Room query exceeded its result capacity"); return; }
        std::array<XrSpaceQueryResultFB, kMaxMembers + 1> batch{};
        output.resultCapacityInput = static_cast<std::uint32_t>(batch.size()); output.results = batch.data();
        if (XR_FAILED(retrieve(session, query->id, &output)) || output.resultCountOutput > batch.size()) {
            Fail("Room query result copy failed"); return;
        }
        for (std::uint32_t i = 0; i < output.resultCountOutput; ++i)
            if (batch[i].space != XR_NULL_HANDLE) owned.insert(batch[i].space);
        if (query->stale) { ReleaseSpaces(); return; }
        for (std::uint32_t i = 0; i < output.resultCountOutput; ++i) {
            const auto& result = batch[i];
            const bool duplicate = std::any_of(query->results.begin(), query->results.end(),
                [&](const auto& previous) { return Key(previous.uuid) == Key(result.uuid) || previous.space == result.space; });
            if (result.space == XR_NULL_HANDLE || !ValidUuid(result.uuid) || duplicate || query->results.size() >= query->limit) {
                Fail("Room query returned invalid or duplicate anchors"); return;
            }
            query->results.push_back(result);
        }
    }
    bool ReadBoundary(Anchor& anchor) {
        XrBoundary2DFB boundary{XR_TYPE_BOUNDARY_2D_FB};
        if (XR_FAILED(get_boundary(session, anchor.space, &boundary)) ||
            boundary.vertexCountOutput < 3 || boundary.vertexCountOutput > kMaxBoundaryVertices) return false;
        anchor.boundary.resize(boundary.vertexCountOutput);
        boundary.vertexCapacityInput = static_cast<std::uint32_t>(anchor.boundary.size()); boundary.vertices = anchor.boundary.data();
        if (XR_FAILED(get_boundary(session, anchor.space, &boundary)) || boundary.vertexCountOutput != anchor.boundary.size()) return false;
        double area = 0;
        for (std::size_t i = 0; i < anchor.boundary.size(); ++i) {
            const auto& p = anchor.boundary[i]; const auto& next = anchor.boundary[(i + 1) % anchor.boundary.size()];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                p.x < anchor.bounds.offset.x - .01F || p.y < anchor.bounds.offset.y - .01F ||
                p.x > anchor.bounds.offset.x + anchor.bounds.extent.width + .01F ||
                p.y > anchor.bounds.offset.y + anchor.bounds.extent.height + .01F) return false;
            area += static_cast<double>(p.x) * next.y - static_cast<double>(next.x) * p.y;
        }
        return std::isfinite(area) && std::abs(area) >= .0001;
    }
    bool ReadSurface(const XrSpaceQueryResultFB& result, bool floor_required, std::optional<Anchor>& output) {
        bool semantic = false;
        if (!Component(result.space, XR_SPACE_COMPONENT_TYPE_SEMANTIC_LABELS_FB, semantic)) return false;
        if (!semantic) return !floor_required;
        if (!EnabledComponent(result.space, XR_SPACE_COMPONENT_TYPE_SEMANTIC_LABELS_FB)) return false;
        XrSemanticLabelsSupportInfoFB support{XR_TYPE_SEMANTIC_LABELS_SUPPORT_INFO_FB};
        support.flags = XR_SEMANTIC_LABELS_SUPPORT_MULTIPLE_SEMANTIC_LABELS_BIT_FB |
            XR_SEMANTIC_LABELS_SUPPORT_ACCEPT_DESK_TO_TABLE_MIGRATION_BIT_FB |
            XR_SEMANTIC_LABELS_SUPPORT_ACCEPT_INVISIBLE_WALL_FACE_BIT_FB;
        support.recognizedLabels = "FLOOR,WALL_FACE,TABLE,INVISIBLE_WALL_FACE,GLOBAL_MESH,OTHER";
        XrSemanticLabelsFB labels{XR_TYPE_SEMANTIC_LABELS_FB}; labels.next = &support;
        std::array<char, 512> text{};
        labels.bufferCapacityInput = static_cast<std::uint32_t>(text.size()); labels.buffer = text.data();
        if (XR_FAILED(get_labels(session, result.space, &labels)) || labels.bufferCountOutput > text.size()) return false;
        const std::string_view names(text.data(), labels.bufferCountOutput);
        const bool floor = HasLabel(names, "FLOOR"), wall = HasLabel(names, "WALL_FACE"), table = HasLabel(names, "TABLE");
        if ((floor ? 1 : 0) + (wall ? 1 : 0) + (table ? 1 : 0) > 1) return false;
        if (floor_required && !floor) return false;
        if (!floor && !wall && !table) {
            if (HasLabel(names, "INVISIBLE_WALL_FACE")) invisible_walls.insert(Key(result.uuid));
            return true;
        }
        Anchor anchor; anchor.space = result.space; anchor.uuid = result.uuid;
        anchor.kind = floor ? RoomSurfaceKind::Floor : wall ? RoomSurfaceKind::Wall : RoomSurfaceKind::Table;
        if (!EnabledComponent(anchor.space, XR_SPACE_COMPONENT_TYPE_BOUNDED_2D_FB) ||
            XR_FAILED(get_bounds(session, anchor.space, &anchor.bounds)) || !BoundsValid(anchor.bounds)) return false;
        if (floor_required && !ReadBoundary(anchor)) return false;
        bool locatable = false;
        if (!Component(anchor.space, XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB, locatable) || !locatable) return false;
        XrSpaceComponentStatusFB status_out{XR_TYPE_SPACE_COMPONENT_STATUS_FB};
        if (XR_FAILED(get_status(anchor.space, XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB, &status_out))) return false;
        anchor.locating = status_out.enabled && !status_out.changePending;
        if (!anchor.locating && !status_out.changePending) {
            XrSpaceComponentStatusSetInfoFB request{XR_TYPE_SPACE_COMPONENT_STATUS_SET_INFO_FB};
            request.componentType = XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB; request.enabled = XR_TRUE; request.timeout = kQueryTimeout;
            const auto result_code = set_status(anchor.space, &request, &anchor.activation);
            if (result_code == XR_ERROR_SPACE_COMPONENT_STATUS_ALREADY_SET_FB) {
                anchor.activation = 0; anchor.locating = EnabledComponent(anchor.space, XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB);
            } else if (XR_FAILED(result_code) || !anchor.activation) {
                if (!floor_required) return false;
                anchor.activation = 0; anchor.activation_failed = true;
            }
        }
        output = std::move(anchor); return true;
    }
    void FinishQuery(XrResult result) {
        if (!query) return;
        if (query->stale) { query.reset(); return; }
        if (XR_FAILED(result)) { Fail("Room query did not complete; Refresh or Scan room"); query.reset(); return; }
        Query finished = std::move(*query); query.reset();
        if (finished.kind != QueryKind::Rooms) {
            std::set<Uuid> wanted, received;
            for (const auto& id : finished.expected) wanted.insert(Key(id));
            for (const auto& item : finished.results) received.insert(Key(item.uuid));
            if (wanted != received) { Fail("Room data incomplete; Refresh or Scan room"); return; }
        }
        if (finished.kind == QueryKind::Rooms) {
            if (finished.results.empty()) { Fail("No saved room; use Scan room"); return; }
            if (finished.results.size() > kMaxRooms) { Fail("Too many saved rooms for collision acquisition"); return; }
            std::vector<XrUuidEXT> ids; std::set<Uuid> floor_ids;
            for (const auto& item : finished.results) {
                if (!EnabledComponent(item.space, XR_SPACE_COMPONENT_TYPE_ROOM_LAYOUT_FB)) {
                    Fail("Saved room layout component is not enabled"); return;
                }
                XrRoomLayoutFB count_only{XR_TYPE_ROOM_LAYOUT_FB};
                const auto count_result = get_layout(session, item.space, &count_only);
                if (XR_FAILED(count_result)) {
                    Fail("Saved room layout count query failed: " + std::to_string(count_result)); return;
                }
                const auto wall_count = count_only.wallUuidCountOutput;
                if (wall_count > kMaxRoomColliders) { Fail("Saved room layout exceeds 64 walls"); return; }
                // XR_FB_scene leaves floor/ceiling UUIDs unspecified when wall
                // capacity is zero. Even an empty wall list needs a filled call
                // with nonzero capacity before reading the floor UUID.
                Room room; room.space = item.space; room.walls.resize(std::max(1U, wall_count));
                XrRoomLayoutFB layout{XR_TYPE_ROOM_LAYOUT_FB};
                layout.wallUuidCapacityInput = static_cast<std::uint32_t>(room.walls.size()); layout.wallUuids = room.walls.data();
                const auto layout_result = get_layout(session, item.space, &layout);
                if (XR_FAILED(layout_result)) {
                    Fail("Saved room layout data query failed: " + std::to_string(layout_result)); return;
                }
                if (layout.wallUuidCountOutput != wall_count) { Fail("Room layout changed during query; Refresh room"); return; }
                if (!ValidUuid(layout.floorUuid)) { Fail("Saved room layout has no floor; Refresh or Scan room"); return; }
                room.floor = layout.floorUuid; room.walls.resize(wall_count);
                if (floor_ids.insert(Key(room.floor)).second) ids.push_back(room.floor);
                rooms.push_back(std::move(room));
            }
            if (StartQuery(QueryKind::Floors, ids)) { state = State::Rooms; status = "Finding the room containing your head"; }
        } else {
            const bool floor_query = finished.kind == QueryKind::Floors;
            auto& destination = floor_query ? floors : surfaces;
            for (const auto& item : finished.results) {
                std::optional<Anchor> anchor;
                if (!ReadSurface(item, floor_query, anchor)) { Fail("Room surface data is invalid or unavailable"); return; }
                if (anchor) destination.push_back(std::move(*anchor));
            }
            if (!floor_query && destination.size() > kMaxRoomColliders) { Fail("Room exceeds 64 collision surfaces"); return; }
            // Match every UUID within each bounded query, then accumulate all
            // batches before publishing or replacing any physics geometry.
            if (!floor_query && next_member < member_ids.size()) { StartMemberBatch(); return; }
            locating_started = now(); state = floor_query ? State::SelectRoom : State::Build; status = "Locating room surfaces";
        }
    }
    bool Locate(Anchor& anchor, XrTime time, XrSpace stage, kin::Pose& pose) {
        if (anchor.activation_failed) return false;
        if (!anchor.locating) {
            anchor.locating = EnabledComponent(anchor.space, XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB);
            if (!anchor.locating) return false;
        }
        XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
        constexpr auto valid = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
        // Static Scene anchors need VALID bits, not necessarily TRACKED bits.
        return XR_SUCCEEDED(xrLocateSpace(anchor.space, stage, time, &location)) &&
            (location.locationFlags & valid) == valid && kin::NormalizePose(Pose(location.pose), pose);
    }
    void Monitor(XrTime time, XrSpace stage) {
        const auto checked_at = now();
        if (!MonitorFresh(checked_at)) { RequireRefresh("Room verification expired; Refresh room"); return; }
        if (!monitor_sweep_active) {
            if (checked_at < next_monitor_sweep) return;
            monitor_sweep_active = true; monitor_cursor = 0;
            next_monitor_sweep = checked_at + kMonitorSweepInterval;
        }
        for (std::size_t checked = 0; checked < kMonitorProbesPerUpdate && monitor_cursor < monitors.size(); ++checked) {
            kin::Pose current;
            if (!Locate(surfaces[monitor_cursor], time, stage, current)) {
                RequireRefresh("Room anchor unavailable; Refresh room"); return;
            }
            const auto& monitor = monitors[monitor_cursor];
            for (std::size_t corner = 0; corner < monitor.anchor_corners.size(); ++corner) {
                const auto point = kin::Compose(current, kin::Pose{monitor.anchor_corners[corner]}).position;
                const auto& captured = monitor.captured_stage_corners[corner];
                const double distance = std::hypot(static_cast<double>(point[0]) - captured[0],
                    static_cast<double>(point[1]) - captured[1], static_cast<double>(point[2]) - captured[2]);
                if (!std::isfinite(distance) || distance > kMonitorMaxCornerDistance) {
                    RequireRefresh("Room alignment changed; Refresh room"); return;
                }
            }
            monitors[monitor_cursor].verified_at = checked_at;
            ++monitor_cursor;
        }
        if (monitor_cursor == monitors.size()) monitor_sweep_active = false;
    }
    void SelectRoom(XrTime time, XrSpace stage, const kin::Pose& head) {
        std::vector<std::optional<kin::Pose>> poses(floors.size());
        for (std::size_t i = 0; i < floors.size(); ++i) {
            kin::Pose pose;
            if (Locate(floors[i], time, stage, pose)) poses[i] = pose;
        }
        unlocated_rooms = 0;
        std::optional<std::size_t> selected;
        for (std::size_t r = 0; r < rooms.size(); ++r) {
            const auto floor = std::find_if(floors.begin(), floors.end(), [&](const auto& f) { return Key(f.uuid) == Key(rooms[r].floor); });
            if (floor == floors.end()) { Fail("A saved room floor is missing"); return; }
            const auto i = static_cast<std::size_t>(floor - floors.begin());
            // Horizon OS may locate only rooms near the current user. This
            // excludes inaccessible candidate rooms, never members of the
            // selected room, which must all pass the later complete query.
            if (!poses[i]) { ++unlocated_rooms; continue; }
            const auto head_local = kin::Compose(kin::Inverse(*poses[i]), head).position;
            kin::Pose rotation_only; rotation_only.rotation = poses[i]->rotation;
            const auto normal = kin::Compose(rotation_only, kin::Pose{{0,0,1}}).position;
            if (std::abs(normal[1]) < .7F) { Fail("Saved floor is not horizontal"); return; }
            const float height = head_local[2] * (normal[1] >= 0 ? 1.F : -1.F);
            if (height < -.1F || height > 4.F || !Contains(floor->boundary, head_local[0], head_local[1])) continue;
            if (selected) { Fail("Overlapping saved rooms; cannot choose room safely"); return; }
            selected = r;
        }
        if (!selected) {
            if (unlocated_rooms) { status = "Waiting for a locatable room containing your head"; return; }
            Fail("Head is outside saved room floors; Refresh or Scan room"); return;
        }
        selected_room = selected; const auto& room = rooms[*selected];
        XrSpaceContainerFB container{XR_TYPE_SPACE_CONTAINER_FB};
        if (!EnabledComponent(room.space, XR_SPACE_COMPONENT_TYPE_SPACE_CONTAINER_FB) ||
            XR_FAILED(get_container(session, room.space, &container)) || !container.uuidCountOutput ||
            container.uuidCountOutput > kMaxMembers) { Fail("Selected room container unavailable or too large"); return; }
        std::vector<XrUuidEXT> ids(container.uuidCountOutput);
        container.uuidCapacityInput = static_cast<std::uint32_t>(ids.size()); container.uuids = ids.data();
        if (XR_FAILED(get_container(session, room.space, &container)) || container.uuidCountOutput != ids.size()) {
            Fail("Selected room container changed; Refresh room"); return;
        }
        // Layout structural members plus container contents; never mix rooms.
        ids.push_back(room.floor); ids.insert(ids.end(), room.walls.begin(), room.walls.end());
        std::set<Uuid> unique; std::vector<XrUuidEXT> members;
        for (const auto& id : ids) {
            if (!ValidUuid(id)) { Fail("Selected room contains an invalid anchor UUID"); return; }
            if (unique.insert(Key(id)).second) members.push_back(id);
        }
        if (members.size() > kMaxMembers) { Fail("Selected room exceeds anchor capacity"); return; }
        member_ids = std::move(members); next_member = 0; StartMemberBatch();
    }
    void Build(XrTime time, XrSpace stage, const kin::Pose& frame) {
        RoomEnvironment candidate; candidate.enabled = true; candidate.revision = snapshot.revision + 1;
        std::vector<AnchorMonitor> candidate_monitors;
        candidate_monitors.reserve(surfaces.size());
        const auto world_from_stage = kin::Inverse(frame);
        for (auto& surface : surfaces) {
            kin::Pose plane; if (!Locate(surface, time, stage, plane)) return;
            const kin::Pose rectangle_center{{surface.bounds.offset.x + surface.bounds.extent.width * .5F,
                                             surface.bounds.offset.y + surface.bounds.extent.height * .5F, 0}};
            auto stage_box = kin::Compose(plane, rectangle_center);
            if (surface.kind != RoomSurfaceKind::Wall) {
                kin::Pose rotation_only; rotation_only.rotation = plane.rotation;
                const auto normal = kin::Compose(rotation_only, kin::Pose{{0,0,1}}).position;
                if (std::abs(normal[1]) < .7F) { Fail("Floor or table surface is not horizontal"); return; }
                // Keep rectangle centre but orient local Z upwards before
                // extending the slab below the captured floor/table surface.
                if (normal[1] < 0) stage_box = kin::Compose(stage_box, kin::Pose{{},{1,0,0,0}});
                stage_box = kin::Compose(stage_box, kin::Pose{{0,0,-kHalfThickness}});
            }
            RoomCollider collider; collider.kind = surface.kind;
            collider.world_from_collider = kin::Compose(world_from_stage, stage_box);
            collider.half_extents = {surface.bounds.extent.width * .5F, surface.bounds.extent.height * .5F, kHalfThickness};
            candidate.colliders.push_back(collider);
            // Capture the exact slab, including rectangle offset and downward
            // extrusion, in anchor-local coordinates. Later pose checks use its
            // corners so even small rotations of large surfaces are detected.
            AnchorMonitor monitor;
            const auto anchor_from_collider = kin::Compose(kin::Inverse(plane), stage_box);
            std::size_t corner = 0;
            for (const float z : {-1.F, 1.F}) for (const float y : {-1.F, 1.F}) for (const float x : {-1.F, 1.F}) {
                const kin::Pose local{{x * collider.half_extents[0], y * collider.half_extents[1], z * collider.half_extents[2]}};
                monitor.anchor_corners[corner] = kin::Compose(anchor_from_collider, local).position;
                monitor.captured_stage_corners[corner] = kin::Compose(plane, kin::Pose{monitor.anchor_corners[corner]}).position;
                ++corner;
            }
            monitor.verified_at = now();
            candidate_monitors.push_back(monitor);
        }
        const auto& room = rooms[*selected_room];
        const auto has = [&](const XrUuidEXT& uuid, RoomSurfaceKind kind) {
            return std::any_of(surfaces.begin(), surfaces.end(), [&](const auto& a) { return Key(a.uuid) == Key(uuid) && a.kind == kind; });
        };
        if (!has(room.floor, RoomSurfaceKind::Floor)) { Fail("Selected room floor is missing"); return; }
        for (const auto& wall : room.walls)
            if (!has(wall, RoomSurfaceKind::Wall) && !invisible_walls.contains(Key(wall))) {
                Fail("Selected room wall is unsupported or missing"); return;
            }
        std::string error;
        if (!ValidateRoomEnvironment(candidate, error)) { Fail("Room collision validation: " + error); return; }
        snapshot = std::move(candidate); capture_frame = frame; capture_stage = stage; frame_set = true;
        monitors = std::move(candidate_monitors); monitor_cursor = 0; monitor_sweep_active = false;
        next_monitor_sweep = now() + kMonitorSweepInterval;
        state = State::Complete; status = ReadyStatus();
        RetainAppliedSpaces(); // Monitoring can invalidate health, never move the snapshot.
    }
};

RoomScene::RoomScene(ClockNow now) : impl_(std::make_unique<Impl>(std::move(now))) {}
RoomScene::~RoomScene() { Shutdown(); }
std::vector<const char*> RoomScene::Extensions() {
    return {XR_FB_SPATIAL_ENTITY_EXTENSION_NAME, XR_FB_SPATIAL_ENTITY_QUERY_EXTENSION_NAME,
            XR_FB_SPATIAL_ENTITY_STORAGE_EXTENSION_NAME, XR_FB_SPATIAL_ENTITY_CONTAINER_EXTENSION_NAME,
            XR_FB_SCENE_EXTENSION_NAME, XR_FB_SCENE_CAPTURE_EXTENSION_NAME};
}
bool RoomScene::Init(XrInstance instance, XrSession session) {
    Shutdown(); auto& p = *impl_;
    if (instance == XR_NULL_HANDLE || session == XR_NULL_HANDLE) return false;
#define LOAD(member, name) Load(instance, name, p.member)
    p.supported = LOAD(query_spaces, "xrQuerySpacesFB") && LOAD(retrieve, "xrRetrieveSpaceQueryResultsFB") &&
        LOAD(enumerate_components, "xrEnumerateSpaceSupportedComponentsFB") && LOAD(get_status, "xrGetSpaceComponentStatusFB") &&
        LOAD(set_status, "xrSetSpaceComponentStatusFB") && LOAD(get_layout, "xrGetSpaceRoomLayoutFB") &&
        LOAD(get_container, "xrGetSpaceContainerFB") && LOAD(get_bounds, "xrGetSpaceBoundingBox2DFB") &&
        LOAD(get_boundary, "xrGetSpaceBoundary2DFB") && LOAD(get_labels, "xrGetSpaceSemanticLabelsFB");
    LOAD(request_capture, "xrRequestSceneCaptureFB");
#undef LOAD
    if (!p.supported) { p.status = "Scene API unavailable"; return false; }
    p.session = session; p.status = "Room collisions off"; return true;
}
void RoomScene::Shutdown() {
    auto& p = *impl_;
    p.Clear(); p.query.reset(); p.capture_id = 0; p.capture_requested = false; p.stage_change_time = 0;
    p.requires_explicit_refresh = false;
    p.session = XR_NULL_HANDLE; p.supported = false; p.enabled = false;
    p.tracked = false; p.permission = false; p.state = Impl::State::Idle; p.status = "Room collisions off";
}
void RoomScene::SetEnabled(bool enabled) {
    auto& p = *impl_; if (p.enabled == enabled) return;
    // An Off transition cancels only a scan that has not reached the platform.
    // Already-started capture_id stays owned until its completion event arrives.
    if (!enabled) p.capture_requested = false;
    p.requires_explicit_refresh = false;
    p.enabled = enabled; p.Clear(); p.state = enabled ? Impl::State::NeedRooms : Impl::State::Idle;
    p.status = enabled ? "Waiting for room permission and tracking" : "Room collisions off";
}
bool RoomScene::Refresh(bool explicit_request) {
    auto& p = *impl_;
    if (!explicit_request) p.LatchExpiredMonitor();
    if (!explicit_request && p.requires_explicit_refresh) { p.status = p.failure; return false; }
    if (explicit_request) p.requires_explicit_refresh = false;
    p.Clear(); p.state = p.enabled ? Impl::State::NeedRooms : Impl::State::Idle;
    p.status = p.enabled ? "Refreshing room" : "Room collisions off";
    return true;
}
bool RoomScene::RequestCapture() {
    auto& p = *impl_;
    if (!p.enabled) { p.status = "Enable room collisions before scanning"; return false; }
    if (!p.supported || !p.session || !p.request_capture) { p.status = "Room scan unavailable"; return false; }
    if (p.capture_id || p.capture_requested) return true;
    if (p.query && p.query->timed_out) { p.status = p.failure; return false; }
    p.requires_explicit_refresh = false;
    p.Clear(); p.capture_requested = true; p.state = p.enabled ? Impl::State::NeedRooms : Impl::State::Idle;
    p.status = "Waiting to start room scan";
    return true;
}
void RoomScene::Invalidate() {
    auto& p = *impl_;
    // Reference events are processed before Update, so recognize an already
    // expired monitor here rather than letting automatic recovery erase it.
    p.LatchExpiredMonitor();
    if (p.requires_explicit_refresh) {
        p.ReleaseSpaces(); p.frame_set = false; p.state = Impl::State::NeedsRefresh; p.status = p.failure; return;
    }
    p.Clear(); p.state = p.enabled ? Impl::State::Invalid : Impl::State::Idle;
    p.status = p.enabled ? "Room frame changed; Refresh room" : "Room collisions off";
}
void RoomScene::OnEvent(const XrEventDataBaseHeader* event) {
    if (!event) return; auto& p = *impl_; if (!p.session) return;
    switch (event->type) {
    case XR_TYPE_EVENT_DATA_SPACE_QUERY_RESULTS_AVAILABLE_FB: {
        const auto& value = *reinterpret_cast<const XrEventDataSpaceQueryResultsAvailableFB*>(event);
        if (p.query && p.query->id == value.requestId) p.Retrieve(); break;
    }
    case XR_TYPE_EVENT_DATA_SPACE_QUERY_COMPLETE_FB: {
        const auto& value = *reinterpret_cast<const XrEventDataSpaceQueryCompleteFB*>(event);
        if (p.query && p.query->id == value.requestId) p.FinishQuery(value.result); break;
    }
    case XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB: {
        const auto& value = *reinterpret_cast<const XrEventDataSpaceSetStatusCompleteFB*>(event);
        if (value.componentType != XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB) break;
        for (auto* collection : {&p.floors, &p.surfaces}) {
            for (auto& anchor : *collection) {
                if (!anchor.activation || anchor.activation != value.requestId || anchor.space != value.space || Key(anchor.uuid) != Key(value.uuid)) continue;
                if (XR_FAILED(value.result) || !value.enabled) {
                    if (collection == &p.floors) {
                        // Candidate rooms may be outside the runtime's current
                        // localization area. Selected member failures stay fatal.
                        anchor.activation = 0; anchor.activation_failed = true; anchor.locating = false;
                        continue;
                    }
                    if (p.state == Impl::State::Complete) p.RequireRefresh("Room anchor unavailable; Refresh room");
                    else p.Fail("Room anchor could not become locatable");
                    return;
                }
                anchor.activation = 0; anchor.locating = true;
            }
        }
        break;
    }
    case XR_TYPE_EVENT_DATA_SCENE_CAPTURE_COMPLETE_FB: {
        const auto& value = *reinterpret_cast<const XrEventDataSceneCaptureCompleteFB*>(event);
        if (!p.capture_id || p.capture_id != value.requestId) break;
        p.capture_id = 0;
        if (XR_FAILED(value.result)) { p.Fail("Room scan cancelled or failed; Refresh or Scan room"); break; }
        Refresh(); break;
    }
    case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
        const auto& value = *reinterpret_cast<const XrEventDataReferenceSpaceChangePending*>(event);
        if (value.session == p.session && value.referenceSpaceType == XR_REFERENCE_SPACE_TYPE_STAGE) {
            // The event precedes the actual origin change. Refresh and enable
            // must not reacquire geometry in the old frame during this interval.
            p.stage_change_time = std::max(p.stage_change_time, value.changeTime);
            Invalidate();
        }
        break;
    }
    case XR_TYPE_EVENT_DATA_EVENTS_LOST:
        p.Fail("XR events lost; reacquire the room after session recovery"); break;
    default: break;
    }
}
void RoomScene::Update(XrTime time, XrSpace stage, const kinematics::Pose& frame,
                       const kinematics::Pose& head, bool tracking_valid, bool permission_granted) {
    auto& p = *impl_; kin::Pose normalized_frame, normalized_head;
    p.tracked = tracking_valid && stage != XR_NULL_HANDLE && time > 0 &&
        kin::NormalizePose(frame, normalized_frame) && kin::NormalizePose(head, normalized_head);
    // Permission/reference/error observations can precede a normal monitor tick.
    // Establish an expired verification latch before those paths alter the state.
    p.LatchExpiredMonitor();
    const bool revoked = p.permission && !permission_granted; p.permission = permission_granted;
    if (revoked && !p.requires_explicit_refresh) {
        p.Clear(); p.state = p.enabled ? Impl::State::NeedRooms : Impl::State::Idle;
    }
    if (!p.supported) { if (p.enabled) p.status = "Scene API unavailable"; return; }
    // A stale query still owns its asynchronous request until the terminal
    // event. Refresh/Scan cannot erase its deadline or spawn concurrent queries.
    if (p.query && !p.query->timed_out && p.now() - p.query->started >= std::chrono::seconds(15)) {
        p.query->timed_out = true;
        const bool queued_scan = p.capture_requested;
        p.capture_requested = false;
        p.Fail(queued_scan ? "Room query stalled; queued scan cancelled. Refresh after completion or restart the app"
                          : "Room query stalled; Refresh after completion or restart the app");
    }
    if (p.query && p.query->timed_out) {
        p.status = p.enabled ? p.failure : "Room collisions off";
        return;
    }
    if (!p.permission) { if (p.enabled || p.capture_requested) p.status = "Scene permission required"; return; }
    if (p.capture_requested && !p.query && !p.capture_id) {
        XrSceneCaptureRequestInfoFB request{XR_TYPE_SCENE_CAPTURE_REQUEST_INFO_FB}; p.capture_requested = false;
        if (XR_FAILED(p.request_capture(p.session, &request, &p.capture_id)) || !p.capture_id) {
            p.capture_id = 0; p.Fail("Cannot open room scan; try again"); return;
        }
    }
    if (p.capture_id) { p.status = "Room scan in progress"; return; }
    if (p.capture_requested) { p.status = "Room scan queued; waiting for previous room query to finish"; return; }
    if (!p.enabled) { p.status = "Room collisions off"; return; }
    if (p.requires_explicit_refresh) { p.state = Impl::State::NeedsRefresh; p.status = p.failure; return; }
    if (!p.tracked) { p.status = "Waiting for valid head tracking"; return; }
    if (time < p.stage_change_time) { p.status = "Waiting for STAGE reference change"; return; }
    p.stage_change_time = 0;
    if (p.frame_set && (p.capture_stage != stage || !SamePose(p.capture_frame, normalized_frame))) { Invalidate(); return; }
    if (p.state == Impl::State::Failed) { p.status = p.failure; return; }
    if (p.state == Impl::State::Invalid) { p.status = "Room frame changed; Refresh room"; return; }
    if (p.state == Impl::State::NeedRooms && p.query) p.status = "Waiting for previous room query to finish";
    if (p.state == Impl::State::Rooms) p.status = "Loading room layouts and floor anchors";
    if (p.state == Impl::State::Members) p.status = "Loading selected room surfaces";
    if (p.state == Impl::State::SelectRoom || p.state == Impl::State::Build) p.status = "Locating room surfaces";
    if (p.state == Impl::State::NeedRooms && !p.query) {
        if (p.StartQuery(Impl::QueryKind::Rooms)) {
            p.state = Impl::State::Rooms; p.capture_stage = stage; p.capture_frame = normalized_frame; p.frame_set = true;
            p.status = "Loading saved room layouts";
        }
    } else if (p.state == Impl::State::SelectRoom) p.SelectRoom(time, stage, normalized_head);
    else if (p.state == Impl::State::Build) p.Build(time, stage, normalized_frame);
    else if (p.state == Impl::State::Complete) {
        p.Monitor(time, stage);
        if (p.state == Impl::State::Complete) p.status = p.ReadyStatus();
    }
    if ((p.state == Impl::State::SelectRoom || p.state == Impl::State::Build) &&
        p.now() - p.locating_started > std::chrono::seconds(15)) p.Fail("Room anchors are not locatable; Refresh room with valid tracking");
}
const RoomEnvironment& RoomScene::Snapshot() const { return impl_->snapshot; }
bool RoomScene::Ready() const {
    return impl_->enabled && impl_->supported && impl_->tracked && impl_->permission &&
           !impl_->requires_explicit_refresh && impl_->state == Impl::State::Complete &&
           impl_->snapshot.enabled && impl_->MonitorFresh(impl_->now());
}
const std::string& RoomScene::Status() const { return impl_->status; }
} // namespace quest_newton
