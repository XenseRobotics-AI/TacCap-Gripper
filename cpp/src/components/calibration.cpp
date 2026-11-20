// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/calibration.hpp>

#include <taccap/error.hpp>
#include <taccap/log.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>

#include <cstdio>
#include <string>
#include <vector>

namespace xense::taccap {

namespace {

// bus::ack_error_code recovers the firmware's echoed-cmd error path, which
// ack.is_nack alone cannot see. Valid here because neither command has a
// legitimate 1-byte non-zero success payload: a successful read is
// CAMERA_FISHEYE_CAL_FULL_SIZE / ENCODER_MAX_CAL_FULL_SIZE bytes, and a
// successful write is the firmware's generic empty response, [0x00].
using bus::ack_error_code;

// Firmware reports a handler error on the cmd=0 pure-ACK path (every follower
// and leader release on hw_v1.1.0; see the response comment in
// protocol_handler.c), which send_cmd raises as NackError before any
// CalNotSet / InvalidParam branch below could see it. Fold it back into an
// AckResponse so both error paths reach ack_error_code().
bus::AckResponse send_cal_cmd(bus::Transport& t, protocol::Cmd cmd,
                              const std::vector<uint8_t>& payload,
                              std::chrono::milliseconds timeout) {
    try {
        return t.send_cmd(cmd, payload, timeout);
    } catch (const NackError& e) {
        bus::AckResponse r{};
        r.cmd        = cmd;
        r.is_nack    = true;
        r.error_code = static_cast<protocol::ErrorCode>(e.code());
        return r;
    }
}

// A write of the reference values stores them as this unit's calibration, and
// every later read takes them for one. Allowed (a stopgap can be deliberate),
// but never silently.
void warn_if_reference_(const protocol::CameraFisheyeCal& cal, const char* what) {
    if (!matches_fisheye_fallback_cal(cal)) return;
    logger()->warn("Calibration::{}: writing the SDK's REFERENCE fisheye intrinsics "
                   "(FISHEYE_FALLBACK_CAL) as this unit's calibration. Every read "
                   "will take them for a real calibration of this unit; calibrate "
                   "the unit and store its own values instead.", what);
}

[[noreturn]] void throw_nack(const char* what, protocol::ErrorCode err) {
    throw ProtocolError(std::string("Calibration::") + what + " NACK: " +
                        protocol::to_string(err));
}

}  // namespace

Calibration::Calibration(bus::Transport& transport) : t_(transport) {}

// ---- Fisheye camera --------------------------------------------------------

std::optional<protocol::CameraFisheyeCal> Calibration::read_fisheye(
        std::chrono::milliseconds timeout) {
    auto ack = send_cal_cmd(t_, protocol::Cmd::CameraFisheyeCal,
                                protocol::encode_camera_fisheye_cal_read(), timeout);
    const auto err = ack_error_code(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_fisheye", err);
    return protocol::decode_camera_fisheye_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_fisheye(const protocol::CameraFisheyeCal& cal,
                                std::chrono::milliseconds timeout) {
    warn_if_reference_(cal, "write_fisheye");
    auto ack = send_cal_cmd(t_, protocol::Cmd::CameraFisheyeCal,
                                protocol::encode_camera_fisheye_cal_write(cal), timeout);
    const auto err = ack_error_code(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_fisheye", err);
}

// ---- Fisheye calibration resolution (V2.10) --------------------------------

bool Calibration::read_fisheye_sized_(std::chrono::milliseconds timeout,
                                      std::optional<FisheyeCalRecord>& out) {
    auto ack = send_cal_cmd(t_, protocol::Cmd::CameraFisheyeCal,
                                protocol::encode_camera_fisheye_cal_read_sized(), timeout);
    const auto err = ack_error_code(ack);
    // Pre-V2.10 firmware answers any op it does not know with InvalidParam;
    // V2.10 answers the sized read with Ok or CalNotSet and nothing else, so
    // this is an unambiguous probe. (A sized WRITE's InvalidParam is not: on
    // V2.10 it also means "implausible values".)
    if (err == protocol::ErrorCode::InvalidParam) {
        note_fisheye_size_support_(false);
        return false;
    }
    if (err == protocol::ErrorCode::CalNotSet) {
        note_fisheye_size_support_(true);
        out.reset();
        return true;
    }
    // Not cached: an unexpected NACK says nothing about the sized ops.
    if (err != protocol::ErrorCode::Ok) throw_nack("read_fisheye_record", err);
    note_fisheye_size_support_(true);
    const auto sized =
        protocol::decode_camera_fisheye_cal_sized(ack.data.data(), ack.data.size());
    out = FisheyeCalRecord{sized.calibration, sized.width, sized.height};
    return true;
}

bool Calibration::supports_fisheye_size(std::chrono::milliseconds timeout) {
    const int cached = fisheye_size_support_.load();
    if (cached >= 0) return cached == 1;
    std::optional<FisheyeCalRecord> ignored;
    return read_fisheye_sized_(timeout, ignored);
}

std::optional<FisheyeCalRecord> Calibration::read_fisheye_record(
        std::chrono::milliseconds timeout) {
    if (fisheye_size_support_.load() != 0) {
        std::optional<FisheyeCalRecord> record;
        if (read_fisheye_sized_(timeout, record)) return record;
    }
    // Older firmware: the params alone, size unknown.
    auto cal = read_fisheye(timeout);
    if (!cal) return std::nullopt;
    return FisheyeCalRecord{*cal, 0, 0};
}

// ---- Dual calibration slots (V2.11) ---------------------------------------

bool Calibration::read_fisheye_slot_(int width, int height,
                                     std::chrono::milliseconds timeout,
                                     std::optional<FisheyeCalRecord>& out) {
    auto ack = send_cal_cmd(t_, protocol::Cmd::CameraFisheyeCal,
                            protocol::encode_camera_fisheye_cal_read_slot(
                                static_cast<uint16_t>(width),
                                static_cast<uint16_t>(height)),
                            timeout);
    const auto err = ack_error_code(ack);
    // Firmware without slots answers op 0x04 as an unknown op (InvalidParam).
    // V2.11 answers InvalidParam only for a size that has no slot, which the
    // callers never send, and LengthMismatch only for a malformed request; a
    // LengthMismatch here can therefore only be a pre-V2.11 table check.
    if (err == protocol::ErrorCode::InvalidParam ||
        err == protocol::ErrorCode::LengthMismatch) {
        fisheye_slot_support_.store(0);
        return false;
    }
    if (err == protocol::ErrorCode::CalNotSet) {
        fisheye_slot_support_.store(1);
        note_fisheye_size_support_(true);   // V2.11 implies the sized ops
        out.reset();
        return true;
    }
    if (err != protocol::ErrorCode::Ok) throw_nack("read_fisheye_slot", err);
    fisheye_slot_support_.store(1);
    note_fisheye_size_support_(true);
    const auto sized =
        protocol::decode_camera_fisheye_cal_slot(ack.data.data(), ack.data.size());
    if ((sized.width != 0 || sized.height != 0) &&
        (sized.width != width || sized.height != height)) {
        throw ProtocolError(
            "Calibration::read_fisheye_slot: asked for the " +
            std::to_string(width) + "x" + std::to_string(height) +
            " slot, the firmware answered with a " + std::to_string(sized.width) +
            "x" + std::to_string(sized.height) + " record");
    }
    out = FisheyeCalRecord{sized.calibration, sized.width, sized.height};
    return true;
}

bool Calibration::supports_fisheye_slots(std::chrono::milliseconds timeout) {
    const int cached = fisheye_slot_support_.load();
    if (cached >= 0) return cached == 1;
    // Firmware that cannot even record a size has no slots either.
    if (fisheye_size_support_.load() == 0) {
        fisheye_slot_support_.store(0);
        return false;
    }
    std::optional<FisheyeCalRecord> ignored;
    return read_fisheye_slot_(FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT, timeout,
                              ignored);
}

std::optional<FisheyeCalRecord> Calibration::read_fisheye_slot(
        int width, int height, std::chrono::milliseconds timeout) {
    if (!is_fisheye_cal_slot_size(width, height)) {
        throw Error("Calibration::read_fisheye_slot: there is no calibration slot "
                    "for " + std::to_string(width) + "x" + std::to_string(height) +
                    " -- only 640x480 and 1920x1080.");
    }
    if (fisheye_slot_support_.load() != 0 && fisheye_size_support_.load() != 0) {
        std::optional<FisheyeCalRecord> record;
        if (read_fisheye_slot_(width, height, timeout, record)) return record;
    }
    // Single-record firmware (V2.10 or older).
    auto record = read_fisheye_record(timeout);
    if (!record) return std::nullopt;
    if (record->size_known()) {
        if (record->width == width && record->height == height) return record;
        return std::nullopt;
    }
    // No size recorded: that is the VGA slot, as on V2.11 (see the header).
    if (width == FISHEYE_CALIB_WIDTH && height == FISHEYE_CALIB_HEIGHT) return record;
    return std::nullopt;
}

void Calibration::clear_fisheye_slot(int width, int height,
                                     std::chrono::milliseconds timeout) {
    if (!is_fisheye_cal_slot_size(width, height)) {
        throw Error("Calibration::clear_fisheye_slot: there is no calibration slot "
                    "for " + std::to_string(width) + "x" + std::to_string(height) +
                    " -- only 640x480 and 1920x1080.");
    }
    if (!supports_fisheye_slots(timeout)) {
        throw Error(
            "Calibration::clear_fisheye_slot: this firmware keeps a single fisheye "
            "record, not per-resolution slots, and cannot clear one. Upgrade the "
            "firmware to 1.3.1 or later (or 1.3.0-rc4+; command set V2.11).");
    }
    auto ack = send_cal_cmd(t_, protocol::Cmd::CameraFisheyeCal,
                            protocol::encode_camera_fisheye_cal_clear_slot(
                                static_cast<uint16_t>(width),
                                static_cast<uint16_t>(height)),
                            timeout);
    const auto err = ack_error_code(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("clear_fisheye_slot", err);
}

void Calibration::write_fisheye_sized(const protocol::CameraFisheyeCal& cal,
                                      int width, int height,
                                      std::chrono::milliseconds timeout) {
    if (!fisheye_cal_size_plausible(cal, width, height)) {
        char buf[384];
        std::snprintf(buf, sizeof(buf),
            "Calibration::write_fisheye_sized: the calibration is not plausible "
            "for %dx%d (fx=%g, fy=%g, cx=%g, cy=%g). Need every value finite, "
            "fx, fy > 0, 1 <= width, height <= 4096 and the principal point "
            "inside the frame (0 < cx < width, 0 < cy < height).",
            width, height, cal.fx, cal.fy, cal.cx, cal.cy);
        throw Error(buf);
    }
    const bool slot_size = is_fisheye_cal_slot_size(width, height);
    if (slot_size && !fisheye_cal_slot_plausible(cal, width, height)) {
        // V2.11's rule, applied whatever the firmware: a principal point this
        // far from the centre means the params belong to another resolution
        // (typically a 640x480 set labelled 1920x1080, or the reverse).
        char buf[512];
        std::snprintf(buf, sizeof(buf),
            "Calibration::write_fisheye_sized: the calibration is not plausible "
            "for %dx%d (fx=%g, fy=%g, cx=%g, cy=%g). At %dx%d the principal "
            "point must lie within 15%% of the frame centre "
            "(%g <= cx <= %g, %g <= cy <= %g) -- the params look like they "
            "belong to another resolution.",
            width, height, cal.fx, cal.fy, cal.cx, cal.cy, width, height,
            width / 2.0 - 0.15 * width, width / 2.0 + 0.15 * width,
            height / 2.0 - 0.15 * height, height / 2.0 + 0.15 * height);
        throw Error(buf);
    }
    if (supports_fisheye_size(timeout)) {
        if (!slot_size && supports_fisheye_slots(timeout)) {
            throw Error(
                "Calibration::write_fisheye_sized: this firmware keeps a "
                "calibration for 640x480 and for 1920x1080 only (command set "
                "V2.11), so a " + std::to_string(width) + "x" +
                std::to_string(height) + " calibration cannot be stored. "
                "Calibrate at one of those two resolutions.");
        }
        warn_if_reference_(cal, "write_fisheye_sized");
        auto ack = send_cal_cmd(t_, protocol::Cmd::CameraFisheyeCal,
                                    protocol::encode_camera_fisheye_cal_write_sized(
                                        cal, static_cast<uint16_t>(width),
                                        static_cast<uint16_t>(height)),
                                    timeout);
        const auto err = ack_error_code(ack);
        if (err != protocol::ErrorCode::Ok) throw_nack("write_fisheye_sized", err);
        return;
    }
    // Older firmware has nowhere to put the size. A 640x480 calibration is
    // what every pre-V2.10 host already assumes, so the legacy write is safe;
    // anything else would be read back as if it were 640x480.
    if (width == FISHEYE_CALIB_WIDTH && height == FISHEYE_CALIB_HEIGHT) {
        write_fisheye(cal, timeout);
        return;
    }
    throw Error(
        "Calibration::write_fisheye_sized: this firmware cannot record the "
        "calibration resolution, so a " + std::to_string(width) + "x" +
        std::to_string(height) + " calibration cannot be stored safely — it "
        "would be taken for 640x480. Upgrade the firmware to 1.3.1 or later (or "
        "a 1.3.0 release candidate; leader and follower, command set V2.10), or "
        "calibrate at 640x480.");
}

// ---- Leader encoder max travel angle ---------------------------------------

std::optional<float> Calibration::read_encoder_max_rad(
        std::chrono::milliseconds timeout) {
    auto ack = send_cal_cmd(t_, protocol::Cmd::EncoderMaxCal,
                                protocol::encode_encoder_max_cal_read(), timeout);
    const auto err = ack_error_code(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_encoder_max_rad", err);
    return protocol::decode_encoder_max_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_encoder_max_rad(float max_rad,
                                        std::chrono::milliseconds timeout) {
    auto ack = send_cal_cmd(t_, protocol::Cmd::EncoderMaxCal,
                                protocol::encode_encoder_max_cal_write(max_rad), timeout);
    const auto err = ack_error_code(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_encoder_max_rad", err);
}

ResolvedFisheyeCal Calibration::resolve_fisheye(std::chrono::milliseconds timeout) {
    std::optional<FisheyeCalRecord> record;
    std::string reason;
    try {
        record = read_fisheye_record(timeout);
    } catch (const ProtocolError& e) {
        // The firmware does not know Cmd::CameraFisheyeCal (0x2B). That command
        // belongs to command set V2.0, which follower 1.1.0 already carries —
        // so this is genuinely old firmware, not merely un-calibrated.
        reason = std::string("the firmware would not answer the fisheye read (")
               + e.what() + ")";
    }
    return resolve_record_(record, std::move(reason));
}

ResolvedFisheyeCal Calibration::resolve_fisheye_for(int width, int height,
                                                    std::chrono::milliseconds timeout) {
    const std::string want = std::to_string(width) + "x" + std::to_string(height);
    std::optional<FisheyeCalRecord> record;
    std::string reason;
    bool from_slot = false;
    try {
        if (is_fisheye_cal_slot_size(width, height) && supports_fisheye_slots(timeout)) {
            from_slot = true;
            record    = read_fisheye_slot(width, height, timeout);
        } else {
            record = read_fisheye_record(timeout);
        }
    } catch (const ProtocolError& e) {
        reason = std::string("the firmware would not answer the fisheye read (")
               + e.what() + ")";
    }
    if (reason.empty() && !record && from_slot) {
        reason = "the wrist lens has never been calibrated at " + want +
                 " (that calibration slot is empty)";
    }
    auto resolved = resolve_record_(record, std::move(reason));
    if (!resolved.is_reference &&
        resolved.calibrated_size != cv::Size(width, height)) {
        const std::string have = std::to_string(resolved.calibrated_size.width) + "x" +
                                 std::to_string(resolved.calibrated_size.height);
        return {FISHEYE_FALLBACK_CAL, true,
                "this unit's wrist calibration belongs to " + have +
                    (resolved.size_inferred ? " (inferred)" : "") + ", not " + want +
                    " -- calibrate at " + want + " to store one",
                cv::Size(FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT), false,
                resolved.calibrated_size};
    }
    return resolved;
}

ResolvedFisheyeCal Calibration::resolve_record_(
        const std::optional<FisheyeCalRecord>& record, std::string reason) {
    if (reason.empty() && !record) {
        reason = "the wrist lens has never been calibrated";
    }
    if (reason.empty() && !is_usable_fisheye_cal(record->calibration)) {
        // An uncalibrated unit answers with an all-zero record rather than a
        // NACK. Remapping with fx = fy = 0 sends every pixel outside the source
        // image, so the frame comes back black.
        reason = "the wrist lens has never been calibrated — the firmware "
                 "answered the read with an empty record";
    }

    cv::Size size;
    bool     inferred = false;
    if (reason.empty()) {
        const auto& c = record->calibration;
        if (record->size_known()) {
            size = cv::Size(record->width, record->height);
            if (!fisheye_cal_size_plausible(c, size.width, size.height)) {
                // The firmware validates a sized write, so this is a damaged
                // or foreign record rather than a calibration to trust.
                reason = "the recorded calibration resolution " +
                         std::to_string(size.width) + "x" +
                         std::to_string(size.height) +
                         " does not fit the stored intrinsics";
            }
        } else if (auto guess = infer_fisheye_cal_size(c)) {
            size     = *guess;
            inferred = true;
            logger()->warn("wrist fisheye calibration carries no resolution (written "
                           "without one, or by firmware older than V2.10); inferred "
                           "{}x{} from its principal point (cx={:.1f}, cy={:.1f}). "
                           "Store it again with write_fisheye_sized() to record the "
                           "size explicitly.", size.width, size.height, c.cx, c.cy);
        } else {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "principal point cx=%.1f, cy=%.1f",
                          c.cx, c.cy);
            reason = std::string("the calibration resolution is unknown and could "
                                 "not be inferred (") + buf + " fits neither "
                     "640x480 nor 1920x1080) — store it again with "
                     "write_fisheye_sized()";
        }
    }

    if (!reason.empty()) {
        return {FISHEYE_FALLBACK_CAL, true, reason,
                cv::Size(FISHEYE_CALIB_WIDTH, FISHEYE_CALIB_HEIGHT), false};
    }
    ResolvedFisheyeCal resolved{record->calibration, false, {}, size, inferred};
    if (matches_fisheye_fallback_cal(record->calibration)) {
        resolved.matches_reference = true;
        logger()->warn("wrist fisheye calibration stored on this unit IS the SDK's "
                       "reference intrinsics (FISHEYE_FALLBACK_CAL), not a calibration "
                       "of this unit -- the factory flashed the reference. "
                       "Rectification is only approximate; recalibrate the unit and "
                       "store its own values with write_fisheye_sized().");
    }
    return resolved;
}

}  // namespace xense::taccap
