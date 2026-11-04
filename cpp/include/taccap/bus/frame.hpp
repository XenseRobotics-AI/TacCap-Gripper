// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// TC-GU-01 framing layer. Implements the wire format defined in the
// firmware: HEAD(0xAA) | ADDR | SEQ | TYPE | CMD | LEN(2 LE) | PAYLOAD
//          | CRC(2 LE) | TAIL(0x55)
//
// Protocol V1.8: the body between HEAD and TAIL (ADDR..CRC) is byte-stuffed
// on the wire — every 0xAA/0x55/0x7D becomes ESC,(byte^0x20) — so HEAD/TAIL
// stay unambiguous delimiters. CRC is computed over the *unstuffed*
// HEAD..PAYLOAD. The layout above is the unstuffed view.
//
// Mirror of third_party/firmware/tc-gu-01/App/protocol/protocol_frame.{h,c}.

#pragma once

#include <taccap/protocol/commands.hpp>

#include <cstdint>
#include <chrono>
#include <array>
#include <atomic>
#include <mutex>
#include <cstddef>
#include <deque>
#include <vector>

namespace xense::taccap::bus {

constexpr uint8_t FRAME_HEAD       = 0xAA;
constexpr uint8_t FRAME_TAIL       = 0x55;
constexpr uint8_t FRAME_ESCAPE     = 0x7D;

constexpr std::size_t FRAME_HEADER_LEN = 7;   // HEAD + ADDR + SEQ + TYPE + CMD + LEN(2)
constexpr std::size_t FRAME_TAIL_LEN   = 3;   // CRC(2) + TAIL(1)
constexpr std::size_t MIN_FRAME_LEN    = FRAME_HEADER_LEN + FRAME_TAIL_LEN;  // 10

// MAX_FRAME_LEN bounds the largest wire-format frame the SDK will
// pack or accept. Set to match firmware (`tc-gu-01/App/protocol/
// protocol_frame.h #define FRAME_MAX_LEN 2304`), sized for V1.3 OTA
// write blocks: payload = 6-byte OtaWriteBlockHeader + up to
// OTA_BLOCK_SIZE=1024 bytes of firmware data = 1030 payload bytes,
// plus 7-byte header + 3-byte tail = 1040 unstuffed; worst-case
// byte-stuffing roughly doubles that, so 2304 leaves headroom.
constexpr std::size_t MAX_FRAME_LEN    = 2304;
constexpr std::size_t MAX_PAYLOAD_LEN  = MAX_FRAME_LEN - MIN_FRAME_LEN;

// CRC-16/MODBUS (init 0xFFFF, poly 0xA001 with reflected input/output).
// Matches firmware protocol_calc_crc().
uint16_t crc16_modbus(const uint8_t* data, std::size_t len) noexcept;

// Byte stuffing: escape FRAME_HEAD / FRAME_TAIL / FRAME_ESCAPE as
// ESC,(byte^0x20). Protocol V1.8 applies this globally to the body between
// HEAD and TAIL, so pack_frame stuffs and try_parse_frame unstuffs (the
// firmware and PC tool do the same). HEAD/TAIL themselves stay raw.
std::vector<uint8_t> stuff_data(const uint8_t* in, std::size_t len);
std::vector<uint8_t> unstuff_data(const uint8_t* in, std::size_t len);

// Decoded frame.
struct Frame {
    protocol::Address    addr;
    uint8_t              seq;
    protocol::FrameType  type;
    protocol::Cmd        cmd;
    std::vector<uint8_t> payload;
    // Host monotonic time of the read completing this frame, not callback time
    // or hardware sample time. Zero for standalone parser-created frames.
    std::chrono::steady_clock::time_point receive_time{};
};

// Build a wire frame.
//
// Throws ProtocolError if payload_len > MAX_PAYLOAD_LEN.
std::vector<uint8_t> pack_frame(
    protocol::Address addr,
    uint8_t           seq,
    protocol::FrameType type,
    protocol::Cmd     cmd,
    const uint8_t*    payload     = nullptr,
    std::size_t       payload_len = 0);

inline std::vector<uint8_t> pack_frame(
        protocol::Address addr, uint8_t seq, protocol::FrameType type,
        protocol::Cmd cmd, const std::vector<uint8_t>& payload) {
    return pack_frame(addr, seq, type, cmd, payload.data(), payload.size());
}

// Single-shot parse outcome.
//   Success      : `frame` populated, advance buffer by `consumed`
//   NeedMoreData : `consumed` is 0; caller should buffer more bytes
//   Resync       : current first byte cannot start a valid frame; caller
//                  should drop one byte (or scan to next 0xAA) and retry.
//                  `consumed` may be >0 if we definitively know how many
//                  bytes are bad, otherwise 1.
enum class ParseStatus { Success, NeedMoreData, Resync };

// Why a parse could not produce a frame at this offset. Only meaningful when
// status == Resync; Success / NeedMoreData always carry ParseReason::None.
//
// This exists to separate "we are mid-stream on a byte boundary" from "we
// found a well-formed frame whose CRC is wrong". Length/framing damage can be
// rejected BEFORE CRC, so zero CRC errors never proves a loss-free link.
// None of these reasons alone identifies the firmware, bridge, or host as
// the source of damage; retain raw bytes for attribution.
enum class ParseReason : uint8_t {
    None = 0,
    NoHead,      // first byte is not HEAD; scanned forward to the next one
    Oversize,    // HEAD with no TAIL inside MAX_FRAME_LEN
    ShortBody,   // unstuffed body below the 8-byte minimum
    BadLength,   // LEN disagrees with the actual unstuffed body length
    BadCrc,      // framing and LEN are self-consistent, CRC is not
};

struct ParseOutcome {
    ParseStatus  status;
    std::size_t  consumed;
    ParseReason  reason = ParseReason::None;
    Frame        frame;   // populated when status == Success
};

ParseOutcome try_parse_frame(const uint8_t* data, std::size_t len);

// Streaming, allocation-friendly frame parser.
// Feed bytes incrementally; pop complete frames as they become available.
// Bounded by max_buffered: if the working buffer grows past that without
// producing a frame, the oldest unsynced bytes are dropped.
class FrameParser {
public:
    explicit FrameParser(std::size_t max_buffered = 64 * 1024);

    void feed(const uint8_t* data, std::size_t len);
    void feed(const std::vector<uint8_t>& bytes);

    // Consume the next ready frame, if any.
    bool try_pop(Frame& out);

    // Number of complete frames currently queued.
    std::size_t pending() const noexcept { return ready_.size(); }
    // Bytes currently held in the resync buffer (not yet a complete frame).
    std::size_t buffered_bytes() const noexcept { return rx_.size(); }

    // Monotonic parser counters. Discarded bytes are counted as resync or
    // buffer overflow; CRC and rejection reasons describe parser outcomes,
    // not the physical location where data was damaged.
    //
    // Not synchronised: only the thread calling feed() may read these. The
    // Transport mirrors them into its own atomics from the reader thread.
    struct Stats {
        uint64_t crc_errors     = 0;  // well-framed CRC mismatch; cause unknown
        uint64_t resync_bytes   = 0;  // bytes discarded without yielding a frame
        uint64_t overflow_bytes = 0;  // bytes dropped by the max_buffered cap
        uint64_t no_head = 0;
        uint64_t oversize = 0;
        uint64_t short_body = 0;
        uint64_t bad_length = 0;
    };
    const Stats& stats() const noexcept { return stats_; }

    // Opt-in bounded evidence, never a user callback or disk write on reader.
    // Input events split large read chunks without omitting bytes. Reject
    // events retain a prefix plus total available size; offsets join them to
    // Input events. Time is host feed time, NOT MCU sample/exposure time.
    struct TraceRecord {
        enum class Kind { Input, Reject };
        Kind kind = Kind::Input;
        ParseReason reason = ParseReason::None;
        std::chrono::steady_clock::time_point host_time{};
        uint64_t offset = 0;
        std::size_t available = 0;
        std::size_t consumed = 0;
        std::size_t size = 0;
        std::array<uint8_t, 1024> bytes{};
    };
    struct TraceSnapshot {
        std::vector<TraceRecord> records;
        uint64_t overwritten_records = 0;
    };
    // Thread-safe. Enabling starts a fresh 256-record ring (~280 KiB).
    // Export after stopping streaming when possible: snapshot holds a mutex.
    void set_trace_enabled(bool enabled);
    TraceSnapshot take_trace();

    // Clears the buffers. Damage counters are monotonic and survive.
    void reset();

private:
    void drain_();
    void trace_(TraceRecord::Kind kind, ParseReason reason, uint64_t offset,
                const uint8_t* data, std::size_t available, std::size_t consumed);
    void rejected_(ParseReason reason);

    std::vector<uint8_t> rx_;
    std::deque<Frame>    ready_;
    std::size_t          max_buffered_;
    Stats                stats_{};
    uint64_t rx_offset_ = 0;
    std::atomic<bool> trace_enabled_{false};
    std::mutex trace_mu_;
    std::vector<TraceRecord> trace_ring_;
    std::size_t trace_next_ = 0, trace_count_ = 0;
    uint64_t trace_overwritten_ = 0;
};

}  // namespace xense::taccap::bus
