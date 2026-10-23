#include "fake_follower.hpp"
#include <taccap/components/imu.hpp>
#include <taccap/error.hpp>
#include <gtest/gtest.h>
#include <condition_variable>

namespace tx = xense::taccap;
namespace tp = tx::protocol;
using namespace taccap_test;

static void append64(std::vector<uint8_t>& p, uint64_t v) {
    for(unsigned i=0;i<8;i++) p.push_back(uint8_t(v>>(i*8)));
}
static std::vector<uint8_t> packet(uint64_t time, uint16_t seq, uint64_t session=0) {
    tp::ImuData raw{}; raw.timestamp_us=uint32_t(time); raw.seq=seq; raw.accel_z=1000;
    auto p=pod_bytes(&raw,sizeof(raw));
    if(session) {append64(p,time);append64(p,session);}
    return p;
}
static tx::ImuSample sample(tx::IMU::Timeline& timeline,uint64_t time,uint16_t seq,uint64_t session=0) {
    auto p=packet(time,seq,session); return timeline.decode(p.data(),p.size());
}
TEST(ImuTime, NativeAcrossMultipleWrapsAndNewBoot) {
    tx::IMU::Timeline t;
    EXPECT_EQ(sample(t,17179869,1,123).mcu_timestamp_us,17179869u);
    auto a=sample(t,(uint64_t(5)<<32)+10,2,123);
    EXPECT_EQ(a.mcu_timestamp_us,(uint64_t(5)<<32)+10);
    EXPECT_TRUE(a.timestamp_valid); EXPECT_FALSE(a.timestamp_discontinuity);
    auto b=sample(t,1000,0,456);
    EXPECT_EQ(b.boot_session_id,456u); EXPECT_EQ(b.mcu_timestamp_us,1000u);
    EXPECT_TRUE(b.timestamp_valid); EXPECT_TRUE(b.timestamp_discontinuity);
    EXPECT_FALSE(sample(t,2000,1,456).timestamp_discontinuity);
}
TEST(ImuTime, NativeReorderedFrameCannotMoveHighWaterMark) {
    tx::IMU::Timeline t; sample(t,20000,2,123);
    EXPECT_FALSE(sample(t,10000,1,123).timestamp_valid);
    EXPECT_FALSE(sample(t,15000,1,123).timestamp_valid);
    EXPECT_TRUE(sample(t,30000,3,123).timestamp_valid);
}
TEST(ImuTime, LegacyWrapAndSequenceWrap) {
    tx::IMU::Timeline t; sample(t,0xfffffff0U,65535);
    auto s=sample(t,0x10,0);
    EXPECT_EQ(s.mcu_timestamp_us,UINT64_C(0x100000010));
    EXPECT_TRUE(s.timestamp_valid); EXPECT_FALSE(s.timestamp_is_64bit);
    EXPECT_EQ(s.boot_session_id,0u);
}
TEST(ImuTime, LegacyDwtRegressionIsNotInventedAsWrap) {
    tx::IMU::Timeline t; sample(t,17173802,15421);
    auto s=sample(t,3933,15422);
    EXPECT_EQ(s.mcu_timestamp_us,3933u);
    EXPECT_FALSE(s.timestamp_valid); EXPECT_TRUE(s.timestamp_discontinuity);
}
TEST(ImuTime, LegacyRestartAndOutOfOrderAreFlagged) {
    tx::IMU::Timeline t; sample(t,500000,200);
    EXPECT_FALSE(sample(t,100,0).timestamp_valid);
    sample(t,300,2); EXPECT_FALSE(sample(t,200,1).timestamp_valid);
}
TEST(ImuTime, LegacyRegressionRemainsInvalidAcrossDuplicatesAndForwardSamples) {
    tx::IMU::Timeline t;
    EXPECT_TRUE(sample(t,17173017,5113).timestamp_valid);
    for (int i=0;i<10;++i) {
        auto s=sample(t,3148,5114);
        EXPECT_FALSE(s.timestamp_valid);
        EXPECT_TRUE(s.timestamp_discontinuity);
    }
    EXPECT_FALSE(sample(t,13148,5115).timestamp_valid);
    // Explicit new tracker establishes a new, unrelated legacy segment.
    tx::IMU::Timeline restarted;
    EXPECT_TRUE(sample(restarted,13148,5115).timestamp_valid);
    // Native time carries a boot identity, so it can safely replace legacy.
    auto native=sample(t,23148,5116,123);
    EXPECT_TRUE(native.timestamp_valid);
    EXPECT_TRUE(native.timestamp_discontinuity);
}
TEST(ImuTime, LegacyLongSilenceIsAmbiguousRatherThanInventedUptime) {
    tx::IMU::Timeline t;
    auto p=packet(100,1); auto start=std::chrono::steady_clock::now();
    t.decode(p.data(),p.size(),start);
    p=packet(200,2);
    auto s=t.decode(p.data(),p.size(),start+std::chrono::minutes(80));
    EXPECT_FALSE(s.timestamp_valid);EXPECT_TRUE(s.timestamp_discontinuity);
}
TEST(ImuTime, RejectsCorruptNativeTimestampAndSession) {
    auto p=packet(UINT64_C(0x100000001),2,123);
    p[0]=2; EXPECT_THROW(tx::IMU::decode(p.data(),p.size()),tx::ProtocolError);
    p[0]=1; std::fill(p.begin()+36,p.end(),0);
    EXPECT_THROW(tx::IMU::decode(p.data(),p.size()),tx::ProtocolError);
    p.resize(43); EXPECT_THROW(tx::IMU::decode(p.data(),p.size()),tx::ProtocolError);
}
TEST(ImuTimePty, NegotiatesAndReadsNativeFrame) {
    Pty pty; tx::bus::Transport transport(base_config(pty.slave_path())); tx::IMU imu(transport);
    std::thread device([&] {
        auto f=pty.expect_frame(); ASSERT_TRUE(f); EXPECT_EQ(f->cmd,tp::Cmd::TimeSession);
        EXPECT_EQ(f->payload.size(),8u);
        std::vector<uint8_t> reply; append64(reply,UINT64_C(0x200000000));append64(reply,123);
        pty.send_response(f->seq,f->cmd,reply);
        f=pty.expect_frame(); ASSERT_TRUE(f); EXPECT_EQ(f->cmd,tp::Cmd::GetImuV2);
        pty.send_response(f->seq,f->cmd,packet(UINT64_C(0x200000010),3,123));
    });
    auto s=imu.read_once(std::chrono::milliseconds{500});
    EXPECT_EQ(s.mcu_timestamp_us,UINT64_C(0x200000010)); EXPECT_EQ(s.boot_session_id,123u);
    device.join();
}
TEST(ImuTimePty, OnlyUnsupportedCommandPermitsFallback) {
    for(auto error : {tp::ErrorCode::InvalidCmd,tp::ErrorCode::SysBusy}) {
        Pty pty; tx::bus::Transport transport(base_config(pty.slave_path())); tx::IMU imu(transport);
        std::thread device([&] {auto f=pty.expect_frame(); ASSERT_TRUE(f); pty.send_nack(f->seq,error);});
        if(error==tp::ErrorCode::InvalidCmd) EXPECT_EQ(imu.negotiate_time(),tp::StreamSrc::Imu);
        else EXPECT_THROW(imu.negotiate_time(),tx::NackError);
        device.join();
    }
}
TEST(ImuTimePty, TimeoutDoesNotSilentlyDowngrade) {
    Pty pty; auto config=base_config(pty.slave_path()); config.max_retries=0;
    tx::bus::Transport transport(config); tx::IMU imu(transport);
    EXPECT_THROW(imu.negotiate_time(std::chrono::milliseconds{20}),tx::TimeoutError);
}
TEST(ImuTimePty, MalformedCapabilityResponseDoesNotDowngrade) {
    Pty pty; tx::bus::Transport transport(base_config(pty.slave_path())); tx::IMU imu(transport);
    std::thread device([&] {auto f=pty.expect_frame(); ASSERT_TRUE(f);
        pty.send_response(f->seq,f->cmd,{0,0,0});});
    EXPECT_THROW(imu.negotiate_time(),tx::ProtocolError);
    device.join();
}
TEST(ImuTimePty, NativeStreamFansOutAndOffRemovesBothFormats) {
    Pty pty; FakeFollower fw(pty); fw.set_imu_v2_supported(true);
    auto g=open_follower(pty);
    std::atomic<unsigned> a{0}, b{0}, motors{0};
    auto first=g->imu().on_data([&](const auto& s) {
        if(s.timestamp_is_64bit && s.timestamp_valid && s.boot_session_id &&
           s.mcu_timestamp_us>UINT32_MAX)++a;
    });
    auto second=g->imu().on_data([&](const auto& s) {if(s.timestamp_is_64bit)++b;});
    auto motor=g->motor().on_status([&](const auto&){++motors;});
    g->start_streaming(100,50);
    EXPECT_EQ(fw.stream_config().source_mask,tp::StreamSrc::MotorStatus|tp::StreamSrc::ImuV2);
    EXPECT_NO_THROW(g->start_streaming(100,50)); // logical source mask stays stable
    for(unsigned i=0;i<100 && (!a||!b||!motors);i++)std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_GT(a.load(),0u);EXPECT_GT(b.load(),0u);EXPECT_GT(motors.load(),0u);
    g->imu().off(first);g->imu().off(second);g->motor().off(motor);
    g->close();
}

TEST(ImuTimePty, DuplicateLegacyFramesDoNotRestoreValidityOrRepeatCallbacks) {
    Pty pty;
    tx::bus::Transport transport(base_config(pty.slave_path()));
    tx::IMU imu(transport);
    std::mutex mutex;
    std::condition_variable ready;
    std::vector<tx::ImuSample> received;
    auto id=imu.on_data([&](const auto& s) {
        std::lock_guard<std::mutex> lock(mutex);
        received.push_back(s);
        ready.notify_all();
    });
    pty.send_data(0,tp::Cmd::GetImu,packet(17173017,5113));
    for (int i=0;i<10;++i)
        pty.send_data(0,tp::Cmd::GetImu,packet(3148,5114));
    pty.send_data(0,tp::Cmd::GetImu,packet(13148,5115));
    {
        std::unique_lock<std::mutex> lock(mutex);
        EXPECT_TRUE(ready.wait_for(lock,std::chrono::seconds(2),[&] {
            return !received.empty() && received.back().seq==5115;
        }));
    }
    imu.off(id);
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(received.size(),3u);
    EXPECT_TRUE(received[0].timestamp_valid);
    EXPECT_FALSE(received[1].timestamp_valid);
    EXPECT_FALSE(received[2].timestamp_valid);
}
