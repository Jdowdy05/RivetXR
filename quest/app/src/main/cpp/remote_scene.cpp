#include "remote_scene.h"
#include "remote_surface.h"
#include "remote_rgb_surface.h"
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>

namespace quest_newton {
namespace {
static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559);
std::uint64_t U(std::span<const std::byte> bytes,std::size_t at,unsigned width){
    std::uint64_t value=0;
    for(unsigned i=0;i<width;++i)value|=static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[at+i]))<<(i*8);
    return value;
}
float F(std::span<const std::byte> bytes,std::size_t at){return std::bit_cast<float>(static_cast<std::uint32_t>(U(bytes,at,4)));}
constexpr auto CrcTable(){
    std::array<std::uint32_t,256> table{};
    for(std::uint32_t i=0;i<256;++i){auto value=i;
        for(unsigned bit=0;bit<8;++bit)value=(value>>1)^((value&1)?0xedb88320U:0U);
        table[i]=value;
    }
    return table;
}
std::uint32_t Crc32(std::span<const std::byte> bytes){
    static constexpr auto table=CrcTable();
    std::uint32_t crc=0xffffffffU;
    for(auto value:bytes)crc=table[(crc^std::to_integer<unsigned char>(value))&255]^(crc>>8);
    return ~crc;
}
using IdentityKey=std::array<std::uint64_t,5>;
IdentityKey Key(const RemoteSceneFrame& frame){return {frame.source_id,frame.world_epoch,frame.calibration_id,static_cast<std::uint64_t>(frame.representation),frame.map_generation};}
}
bool DecodeRemoteScene(std::span<const std::byte> packet,RemoteSceneFrame& output,std::string& error){
    const auto reject=[&](const char* message){error=message;return false;};
    if(packet.size()<kRemoteSceneHeaderBytes||packet.size()>kRemoteSceneMaxPacketBytes)return reject("Remote packet size is outside bounds");
    const auto version=U(packet,4,2);
    const std::size_t header=version==1?kRemoteSceneHeaderBytes:version==2?kRemoteSurfaceHeaderBytes:kRemoteRgbHeaderBytes;
    if(U(packet,0,4)!=0x4e435352U||(version!=1&&version!=2&&version!=3)||U(packet,6,2)!=header||packet.size()<header)
        return reject("Invalid RSCN version or header");
    if(U(packet,92,4)||U(packet,124,4))return reject("Unknown RSCN reserved fields");
    RemoteSceneFrame next;next.wire_version=static_cast<std::uint32_t>(version);
    next.source_id=U(packet,8,8);next.world_epoch=U(packet,16,8);next.sequence=U(packet,24,8);next.calibration_id=U(packet,32,8);
    if(!next.source_id||!next.world_epoch||!next.sequence||!next.calibration_id)return reject("RSCN identity and sequence must be nonzero");
    next.capture_start_ns=U(packet,40,8);next.capture_end_ns=U(packet,48,8);next.produced_ns=U(packet,56,8);
    if(!next.produced_ns)return reject("RSCN production timestamp must be nonzero");
    const auto count=static_cast<std::uint32_t>(U(packet,64,4));
    next.expected_camera_mask=static_cast<std::uint32_t>(U(packet,68,4));
    next.contributing_mask=static_cast<std::uint32_t>(U(packet,72,4));next.flags=static_cast<std::uint32_t>(U(packet,76,4));
    next.voxel_size_m=F(packet,80);next.max_capture_skew_ms=static_cast<std::uint32_t>(U(packet,84,4));
    if(version==1&&(count>kRemoteSceneMaxPoints||packet.size()!=kRemoteSceneHeaderBytes+static_cast<std::size_t>(count)*kRemoteScenePointBytes))
        return reject("RSCN point count does not match payload length");
    if(!next.expected_camera_mask||(next.expected_camera_mask&~(version==3?63U:31U))||(next.contributing_mask&~next.expected_camera_mask))
        return reject("Invalid RSCN camera coverage");
    if((next.flags&~15U)||((next.flags&kRemoteScenePartial)!=0)!=(next.expected_camera_mask!=next.contributing_mask))
        return reject("Invalid RSCN source or partial-coverage flags");
    if((version==1&&(!std::isfinite(next.voxel_size_m)||next.voxel_size_m<.001F||next.voxel_size_m>.1F))||
       (version>=2&&(count||U(packet,80,4)))||
       next.max_capture_skew_ms<1||next.max_capture_skew_ms>100)return reject("Invalid RSCN voxel size or capture-skew limit");
    if(!next.contributing_mask){
        if(count||next.capture_start_ns||next.capture_end_ns)return reject("No-camera RSCN frame must have empty geometry and capture times");
    }else if(!next.capture_start_ns||next.capture_end_ns<next.capture_start_ns||next.produced_ns<next.capture_end_ns||
             next.capture_end_ns-next.capture_start_ns>static_cast<std::uint64_t>(next.max_capture_skew_ms)*1000000ULL||
             next.produced_ns-next.capture_end_ns>2000000000ULL)return reject("Invalid or stale RSCN producer-clock capture interval");
    for(std::size_t axis=0;axis<3;++axis){
        const auto value=F(packet,96+axis*4);
        if(!std::isfinite(value)||std::abs(value)>100.F)return reject("Invalid RSCN observer position");
        next.suggested_observer.position[axis]=value;
    }
    double norm=0;
    for(std::size_t axis=0;axis<4;++axis){
        const auto value=F(packet,108+axis*4);
        if(!std::isfinite(value))return reject("Invalid RSCN observer rotation");
        next.suggested_observer.rotation[axis]=value;norm+=static_cast<double>(value)*value;
    }
    if(std::abs(std::sqrt(norm)-1.)>1e-3||!kinematics::NormalizePose(next.suggested_observer,next.suggested_observer))
        return reject("RSCN observer rotation must be unit length");
    if(Crc32(packet.subspan(header))!=U(packet,88,4))return reject("RSCN payload CRC mismatch");
    if(version==3){
        if(!DecodeRemoteRgbSurface(packet,next,error))return false;
        output=std::move(next);error.clear();return true;
    }
    if(version==2){
        if(!DecodeRemoteSurface(packet,next,error))return false;
        output=std::move(next);error.clear();return true;
    }
    const auto cameras=std::popcount(next.contributing_mask);
    next.points.reserve(count);
    for(std::uint32_t index=0;index<count;++index){
        const auto at=kRemoteSceneHeaderBytes+static_cast<std::size_t>(index)*kRemoteScenePointBytes;
        RemoteScenePoint point;
        for(std::size_t axis=0;axis<3;++axis){
            const auto value=F(packet,at+axis*4);
            if(!std::isfinite(value)||std::abs(value)>100.F)return reject("Invalid RSCN point position");
            point.position[axis]=value;
        }
        point.grayscale=static_cast<std::uint8_t>(U(packet,at+12,1));point.support=static_cast<std::uint8_t>(U(packet,at+13,1));
        point.radius_mm=static_cast<std::uint16_t>(U(packet,at+14,2));
        if(point.support<1||point.support>5||point.support>cameras||point.radius_mm<1||point.radius_mm>50)
            return reject("Invalid RSCN camera-support count or point radius");
        next.points.push_back(point);
    }
    output=std::move(next);error.clear();return true;
}
double RemoteSceneAgeMs(const RemoteSceneSnapshot& snapshot,RemoteSceneTime now){
    if(!snapshot.frame||now<snapshot.received_at)return -1;
    // Convert before subtracting so even extreme local test times do not overflow
    // the clock's signed integral duration. Producer timestamps are never used.
    const auto current=std::chrono::duration<long double,std::milli>(now.time_since_epoch()).count();
    const auto received=std::chrono::duration<long double,std::milli>(snapshot.received_at.time_since_epoch()).count();
    return static_cast<double>(current-received);
}
bool RemoteSceneFresh(const RemoteSceneSnapshot& snapshot,RemoteSceneTime now,std::chrono::milliseconds max_age){
    const auto age=RemoteSceneAgeMs(snapshot,now);
    return max_age.count()>=0&&age>=0&&age<=static_cast<double>(max_age.count());
}
struct RemoteSceneMailbox::Impl {
    std::mutex mutex;
    RemoteSceneSnapshot snapshot;
    std::map<IdentityKey,std::uint64_t> sequences;
};
RemoteSceneMailbox::RemoteSceneMailbox():impl_(std::make_unique<Impl>()){}
RemoteSceneMailbox::~RemoteSceneMailbox()=default;
std::uint64_t RemoteSceneMailbox::Reset(){
    const std::lock_guard lock(impl_->mutex);auto& snapshot=impl_->snapshot;
    if(snapshot.stream_generation==std::numeric_limits<std::uint64_t>::max()||snapshot.publication==std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("Remote mailbox identity counter exhausted");
    snapshot.frame.reset();snapshot.received_at={};snapshot.identity_revision=0;++snapshot.stream_generation;++snapshot.publication;
    impl_->sequences.clear();return snapshot.stream_generation;
}
bool RemoteSceneMailbox::Publish(std::span<const std::byte> packet,RemoteSceneTime received_at,std::uint64_t generation,std::string& error){
    {
        const std::lock_guard lock(impl_->mutex);
        if(!generation||generation!=impl_->snapshot.stream_generation){error="Retired remote receiver generation";return false;}
    }
    RemoteSceneFrame decoded;if(!DecodeRemoteScene(packet,decoded,error))return false;
    auto frame=std::make_shared<const RemoteSceneFrame>(std::move(decoded));
    const std::lock_guard lock(impl_->mutex);auto& snapshot=impl_->snapshot;
    if(generation!=snapshot.stream_generation){error="Retired remote receiver generation";return false;}
    const auto key=Key(*frame);const auto previous=impl_->sequences.find(key);
    if(previous!=impl_->sequences.end()&&frame->sequence<=previous->second){error="RSCN sequence did not increase within its identity";return false;}
    if(previous==impl_->sequences.end()&&impl_->sequences.size()>=64){error="Remote identity history is full; restart the source";return false;}
    if(snapshot.frame&&received_at<snapshot.received_at){error="Remote local receipt time moved backwards";return false;}
    if(snapshot.publication==std::numeric_limits<std::uint64_t>::max()){error="Remote publication counter exhausted";return false;}
    const bool identity_changed=!snapshot.frame||snapshot.frame->Identity()!=frame->Identity();
    if(identity_changed&&snapshot.identity_revision==std::numeric_limits<std::uint64_t>::max()){error="Remote identity revision exhausted";return false;}
    impl_->sequences[key]=frame->sequence;
    if(identity_changed)++snapshot.identity_revision;
    snapshot.frame=std::move(frame);snapshot.received_at=received_at;++snapshot.publication;
    error.clear();return true;
}
RemoteSceneSnapshot RemoteSceneMailbox::Read()const{const std::lock_guard lock(impl_->mutex);return impl_->snapshot;}
} // namespace quest_newton
