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

[[noreturn]] void throw_nack(const char* what, protocol::ErrorCode err) {
    throw ProtocolError(std::string("Calibration::") + what + " NACK: " +
                        protocol::to_string(err));
}

}  // namespace

Calibration::Calibration(bus::Transport& transport) : t_(transport) {}

// ---- Fisheye camera --------------------------------------------------------

std::optional<protocol::CameraFisheyeCal> Calibration::read_fisheye(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
                           protocol::encode_camera_fisheye_cal_read(), timeout);
    const auto err = ack_error_code(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_fisheye", err);
    return protocol::decode_camera_fisheye_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_fisheye(const protocol::CameraFisheyeCal& cal,
                                std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
                           protocol::encode_camera_fisheye_cal_write(cal), timeout);
    const auto err = ack_error_code(ack);
    if (err != protocol::ErrorCode::Ok) throw_nack("write_fisheye", err);
}

// ---- Fisheye calibration resolution (V2.10) --------------------------------

bool Calibration::read_fisheye_sized_(std::chrono::milliseconds timeout,
                                      std::optional<FisheyeCalRecord>& out) {
    auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
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
    if (supports_fisheye_size(timeout)) {
        auto ack = t_.send_cmd(protocol::Cmd::CameraFisheyeCal,
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
        "would be taken for 640x480. Upgrade the firmware to leader 1.2.8 / "
        "follower 1.2.18 or later (command set V2.10), or calibrate at 640x480.");
}

// ---- Leader encoder max travel angle ---------------------------------------

std::optional<float> Calibration::read_encoder_max_rad(
        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::EncoderMaxCal,
                           protocol::encode_encoder_max_cal_read(), timeout);
    const auto err = ack_error_code(ack);
    if (err == protocol::ErrorCode::CalNotSet) return std::nullopt;
    if (err != protocol::ErrorCode::Ok) throw_nack("read_encoder_max_rad", err);
    return protocol::decode_encoder_max_cal(ack.data.data(), ack.data.size());
}

void Calibration::write_encoder_max_rad(float max_rad,
                                        std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::EncoderMaxCal,
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
    return {record->calibration, false, {}, size, inferred};
}

}  // namespace xense::taccap
