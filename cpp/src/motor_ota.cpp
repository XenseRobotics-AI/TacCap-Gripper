// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/motor_ota.hpp>

#include <taccap/components/motor.hpp>
#include <taccap/error.hpp>
#include <taccap/log.hpp>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace xense::taccap {

namespace {

constexpr uint8_t kCommGetDeviceId = 0;
constexpr uint8_t kCommOtaStart = 11;
constexpr uint8_t kCommOtaInfo = 12;
constexpr uint8_t kCommOtaData = 13;
constexpr uint8_t kCommOtaEnd = 14;

constexpr uint8_t kUidReplyDst = 0xFE;  // type 0 answers to 0xFE, not the host

// Reply id: mode in 28..24, result in 23..16, motor id in 15..8, host in 7..0.
constexpr uint32_t kModeMask = 0x1Fu << 24;
constexpr uint32_t kLow16 = 0xFFFFu;

uint8_t reply_mode(uint32_t id) { return static_cast<uint8_t>((id >> 24) & 0x1F); }
uint8_t reply_result(uint32_t id) { return static_cast<uint8_t>((id >> 16) & 0xFF); }

std::vector<uint8_t> le32_pair(uint32_t a, uint32_t b) {
    std::vector<uint8_t> out(8);
    for (int i = 0; i < 4; ++i) {
        out[i] = static_cast<uint8_t>(a >> (8 * i));
        out[4 + i] = static_cast<uint8_t>(b >> (8 * i));
    }
    return out;
}

}  // namespace

MotorOtaSession::MotorOtaSession(Motor& motor, uint8_t motor_can_id, Options opts)
    : xfer_([&motor](uint32_t id, const std::vector<uint8_t>& d, uint16_t t,
                     uint32_t mask, uint32_t value) {
          return motor.can_ext_xfer(id, d, t, mask, value);
      }),
      motor_(&motor), can_id_(motor_can_id), opts_(opts) {}

MotorOtaSession::MotorOtaSession(Motor& motor, uint8_t motor_can_id)
    : MotorOtaSession(motor, motor_can_id, Options{}) {}

MotorOtaSession::MotorOtaSession(Xfer xfer, uint8_t motor_can_id, Options opts)
    : xfer_(std::move(xfer)), can_id_(motor_can_id), opts_(opts) {}

MotorOtaSession::MotorOtaSession(Xfer xfer, uint8_t motor_can_id)
    : MotorOtaSession(std::move(xfer), motor_can_id, Options{}) {}

void MotorOtaSession::preflight(const std::string& expected_model) {
    if (motor_ == nullptr) {
        throw std::logic_error("MotorOtaSession::preflight needs a session built on a Motor");
    }
    if (motor_->get_protocol() != protocol::MotorProtocol::Private) {
        throw ProtocolError(
            "motor OTA: the motor is on MIT; it does not answer OTA frames there "
            "(measured). switch_protocol(Private), then power-cycle -- cut 24 V "
            "for ~2 s, USB may stay in -- and retry");
    }
    const auto model = motor_->get_model();
    const std::string name(model.name, ::strnlen(model.name, sizeof(model.name)));
    if (name != expected_model) {
        throw ProtocolError("motor OTA: this gripper records a " + name +
                            " motor, the image is for " + expected_model +
                            ". The motor would accept it -- the protocol has no "
                            "model check -- so this refuses instead");
    }
    if (!model.from_flash) {
        logger()->warn("motor OTA: the motor model {} is the firmware's compile-time "
                       "default, not a stored record -- confirm the nameplate", name);
    }
}

uint32_t MotorOtaSession::make_id(uint8_t mode, uint16_t data16) const {
    return (static_cast<uint32_t>(mode & 0x1F) << 24) |
           (static_cast<uint32_t>(data16) << 8) | can_id_;
}

MotorOtaSession::Reply MotorOtaSession::exchange(uint8_t mode, uint16_t data16,
                                                 const std::vector<uint8_t>& payload,
                                                 uint16_t timeout_ms, uint32_t mask,
                                                 uint32_t value, const char* what) {
    const uint32_t id = make_id(mode, data16);
    const int attempts = (mode == kCommOtaStart) ? opts_.start_attempts : opts_.max_attempts;
    for (int attempt = 1; attempt <= attempts; ++attempt) {
        const auto r = xfer_(id, payload, timeout_ms, mask, value);
        if (r.status == protocol::motor_can_xfer_status::Ok) {
            return Reply{reply_mode(r.ext_id), reply_result(r.ext_id), r};
        }
        if (r.status == protocol::motor_can_xfer_status::TxFailed) {
            throw IoError(std::string("motor OTA ") + what +
                          ": the frame never left the MCU (CAN TX failed)", EIO);
        }
        logger()->warn("motor OTA {}: no reply (attempt {}/{})", what, attempt, attempts);
    }
    throw TimeoutError(std::string("motor OTA ") + what + ": no reply after " +
                       std::to_string(attempts) +
                       " attempts. RobStride's advice is to power the motor off and "
                       "start again -- the update restarts from zero",
                       ETIMEDOUT);
}

MotorOtaSession::Reply MotorOtaSession::handshake(uint8_t mode, uint16_t data16,
                                                  const std::vector<uint8_t>& payload,
                                                  const char* what) {
    const uint32_t mask = kModeMask | kLow16;
    const uint32_t value = (static_cast<uint32_t>(mode) << 24) |
                           (static_cast<uint32_t>(can_id_) << 8) | opts_.host_id;
    return exchange(mode, data16, payload, opts_.handshake_timeout_ms, mask, value, what);
}

std::array<uint8_t, 8> MotorOtaSession::read_uid() {
    const uint32_t mask = kModeMask | kLow16;
    const uint32_t value = (static_cast<uint32_t>(kCommGetDeviceId) << 24) |
                           (static_cast<uint32_t>(can_id_) << 8) | kUidReplyDst;
    const auto rep = exchange(kCommGetDeviceId, opts_.host_id, std::vector<uint8_t>(8, 0),
                              opts_.handshake_timeout_ms, mask, value, "uid");
    if (rep.raw.dlc < 8) {
        throw ProtocolError("motor OTA uid: short reply");
    }
    std::array<uint8_t, 8> uid{};
    std::memcpy(uid.data(), rep.raw.data, 8);
    return uid;
}

void MotorOtaSession::update_from_file(const std::string& path,
                                       ProgressCallback on_progress) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw IoError("motor OTA: cannot open " + path, errno);
    }
    std::vector<uint8_t> image((std::istreambuf_iterator<char>(f)),
                               std::istreambuf_iterator<char>());
    update_from_bytes(image, std::move(on_progress));
}

void MotorOtaSession::update_from_bytes(const std::vector<uint8_t>& image,
                                        ProgressCallback on_progress) {
    const auto size = static_cast<uint32_t>(image.size());
    if (size == 0 || size > kMaxImageSize) {
        throw std::invalid_argument("motor OTA: image size " + std::to_string(size) +
                                    " outside 1.." + std::to_string(kMaxImageSize));
    }
    const uint32_t packs = (size + 7) / 8;

    const auto uid = read_uid();
    logger()->info("motor OTA: uid {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}, "
                   "{} B in {} packs", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5],
                   uid[6], uid[7], size, packs);

    // start: the motor restarts into its bootloader before it answers (~4 s
    // measured), hence Options::start_attempts. From here on it is not running
    // its application.
    auto rep = handshake(kCommOtaStart, opts_.host_id,
                         std::vector<uint8_t>(uid.begin(), uid.end()), "start");
    if (rep.result != 0) {
        throw ProtocolError("motor OTA start: motor refused, result " +
                            std::to_string(rep.result));
    }

    rep = handshake(kCommOtaInfo, opts_.host_id, le32_pair(size, packs), "info");
    if (rep.result != 0) {
        throw ProtocolError("motor OTA info: motor refused, result " +
                            std::to_string(rep.result));
    }

    // Data replies: type 13 per OTA.py, 11 per the PDF table. 11 and 13 agree
    // on mode bits 0, 3 and 4 (0b01001), so a mask over those bits admits
    // both while still excluding the type-2 status replies the MCU's idle
    // polling can draw from a motor still running its application.
    const uint32_t data_mask = (0x19u << 24) | kLow16;
    const uint32_t data_value = (0x09u << 24) |
                                (static_cast<uint32_t>(can_id_) << 8) | opts_.host_id;

    int resumes = 0;
    auto rewind = [&](uint32_t to, uint32_t at, const char* why) {
        if (to > packs) {
            throw ProtocolError(std::string("motor OTA ") + why + ": motor asked to resume at " +
                                std::to_string(to) + ", beyond the " +
                                std::to_string(packs) + " packs sent");
        }
        if (++resumes > opts_.max_resumes) {
            throw ProtocolError("motor OTA: more than " +
                                std::to_string(opts_.max_resumes) +
                                " resume requests -- giving up");
        }
        logger()->warn("motor OTA {}: motor asked to resume at pack {} (was at {})",
                       why, to, at);
        return to;
    };

    uint32_t cnt = 0;
    for (;;) {
        while (cnt < packs) {
            std::vector<uint8_t> chunk(8, 0xFF);
            const uint32_t off = cnt * 8;
            const uint32_t n = std::min<uint32_t>(8, size - off);
            std::memcpy(chunk.data(), image.data() + off, n);

            const auto r = exchange(kCommOtaData, static_cast<uint16_t>(cnt), chunk,
                                    opts_.data_timeout_ms, data_mask, data_value, "data");
            if (r.mode != kCommOtaData && r.mode != kCommOtaStart) {
                throw ProtocolError("motor OTA data: unexpected reply type " +
                                    std::to_string(r.mode));
            }
            if (r.result == 0) {
                ++cnt;
                if (on_progress && (cnt % 256 == 0)) {
                    on_progress(cnt, packs);
                }
                continue;
            }
            if (r.raw.dlc < 2) {
                throw ProtocolError("motor OTA data: failure reply without a resume index");
            }
            const uint32_t to = static_cast<uint32_t>(r.raw.data[0]) |
                                (static_cast<uint32_t>(r.raw.data[1]) << 8);
            cnt = rewind(to, cnt, "data");
        }

        // end: the reply's data field carries the motor's own position when it
        // disagrees that the image is complete -- rewind and send the rest.
        auto end = handshake(kCommOtaEnd, 0, le32_pair(packs, 0), "end");
        if (end.result == 0) {
            break;
        }
        if (end.raw.dlc < 4) {
            throw ProtocolError("motor OTA end: failure reply without a position");
        }
        uint32_t to = 0;
        std::memcpy(&to, end.raw.data, 4);
        if (to >= packs) {
            throw ProtocolError("motor OTA end: motor reports failure at pack " +
                                std::to_string(to) + " of " + std::to_string(packs));
        }
        cnt = rewind(to, packs, "end");
    }

    if (on_progress) {
        on_progress(packs, packs);
    }
    logger()->info("motor OTA: end acknowledged, motor restarts ({} resumes)", resumes);
}

}  // namespace xense::taccap
