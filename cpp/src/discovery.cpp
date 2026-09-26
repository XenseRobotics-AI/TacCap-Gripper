// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/discovery.hpp>
#include <taccap/bus/transport.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>
#include <taccap/protocol/commands.hpp>
#include <taccap/protocol/payloads.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <optional>
#include <regex>
#include <system_error>

namespace xense::taccap::discovery {

namespace fs = std::filesystem;

namespace {

bool is_digit_(char c) noexcept { return c >= '0' && c <= '9'; }

}  // namespace

std::optional<Side> side_from_serial(const std::string& s) noexcept {
    for (auto it = s.rbegin(); it != s.rend(); ++it) {
        if (is_digit_(*it)) {
            return ((*it - '0') % 2 == 1) ? Side::Left : Side::Right;
        }
    }
    return std::nullopt;
}

ParsedSerial parse_serial(const std::string& s) noexcept {
    ParsedSerial out;
    out.raw = s;

    // Full TacCap grammar:
    //   product (4 alpha + 2 digit) | batch (1 alpha + 2 digit) |
    //   line (1 alpha: Z=R&D, A=production) | sequence (4 digit) |
    //   patch (optional m|s).  e.g.  TCGU01 A24 Z 0001 m   /   GSPS01 A24 Z 0001
    static const std::regex re(
        R"(^([A-Za-z]{4}\d{2})([A-Za-z]\d{2})([A-Za-z])(\d{4})([mMsS]?)$)");
    std::smatch m;
    if (std::regex_match(s, m, re)) {
        out.product  = m[1].str();
        out.batch    = m[2].str();
        out.line     = m[3].str()[0];
        out.sequence = m[4].str();
        const char last = out.sequence.back();
        out.side = ((last - '0') % 2 == 1) ? Side::Left : Side::Right;
        const std::string patch = m[5].str();
        if (!patch.empty()) {
            const char p = static_cast<char>(
                std::tolower(static_cast<unsigned char>(patch[0])));
            out.role = (p == 'm') ? Role::Leader : Role::Follower;
        }
        out.valid = true;
        return out;
    }

    // Not a full TacCap SN (legacy "SN0000002", empty, vendor string, ...).
    // Recover what we can so callers still get a usable side/role: the last
    // digit decides side, a trailing m|s decides role.
    out.side = side_from_serial(s);
    if (!s.empty()) {
        const char last = static_cast<char>(
            std::tolower(static_cast<unsigned char>(s.back())));
        if (last == 'm') out.role = Role::Leader;
        else if (last == 's') out.role = Role::Follower;
    }
    return out;
}

namespace {

// Extract SN from "/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5CXXXXXXXX-if02".
// Returns the SN portion or empty string on miss.
std::string parse_ch343_serial(const std::string& path) {
    static const std::regex re(
        R"(usb-1a86_USB_Dual_Serial_([^-]+)-if02$)");
    std::smatch m;
    if (std::regex_search(path, m, re)) return m[1].str();
    return {};
}

}  // namespace

// ---- scanners -------------------------------------------------------------

std::vector<McuEndpoint> scan_mcus() {
    std::vector<McuEndpoint> out;
    std::error_code ec;
    fs::path by_id = "/dev/serial/by-id";
    if (!fs::is_directory(by_id, ec)) return out;
    for (auto& e : fs::directory_iterator(by_id, ec)) {
        if (ec) break;
        const std::string p = e.path().string();
        // Only the if02 interface of the CH343 dual-serial is the MCU control
        // link. The chip SN is kept for identification only — it is NOT used to
        // decide the side (that comes from the firmware SN / GetDevType later).
        const auto sn = parse_ch343_serial(p);
        if (sn.empty()) continue;
        out.push_back({p, sn});
    }
    return out;
}

std::vector<GripperEndpoints> scan_all() {
    // One MCU board (CH343 if02 entry) = one gripper unit. The wrist camera
    // and OG visuotactile sensors are owned by an external camera service
    // now, so discovery is MCU-only — no USB-hub grouping needed.
    //
    // Side priority — firmware sources ONLY (never the CH343 chip SN, whose
    // parity is unrelated to the board's left/right identity and can be wrong):
    //   1. TacCap SN (GetSn 0x04 / parse_serial): the burned SN encodes the
    //      side in its sequence digit — for an m/s-suffixed gripper SN
    //      (TCGU01A24Z0001m) that is the *second-to-last character* of the
    //      string (the suffix m/s is last). Authoritative; retried on cold start.
    //   2. GetDevType (0x06): the flash-burned LEFT/RIGHT byte — fallback for
    //      units whose SN isn't burned / readable (e.g. firmware too old for GetSn).
    //   3. Neither answered → Side::Unknown. We do NOT guess from the chip SN.
    // Role (leader/follower) only comes from the SN patch suffix (m/s).
    std::vector<GripperEndpoints> out;
    for (auto& mcu : scan_mcus()) {
        // Open a transient transport and probe the firmware. One-shot startup
        // cost — closed before LeaderGripper / FollowerGripper open their own
        // Transport on the same device. ack_timeout/retries are kept low here
        // because both probes are best-effort (old firmware may not answer).
        std::string fw_sn;
        std::optional<Side> dev_side;   // from GetDevType (authoritative)
        bool in_use = false;
        try {
            bus::Transport::Config cfg{};
            cfg.serial.device           = mcu.device;
            cfg.serial.baudrate         = 3'000'000;
            cfg.serial.read_timeout_ms  = 1;
            cfg.serial.write_timeout_ms = 1000;
            cfg.peer                    = protocol::Address::MCU;
            cfg.ack_timeout             = std::chrono::milliseconds(500);
            cfg.max_retries             = 1;
            bus::Transport t(cfg);
            // If the firmware is still streaming from a previous host process
            // (Ctrl+C / abort, or a sibling publisher we just killed), the rx
            // pipe is full of DATA frames and the next ACK gets buried until
            // they drain. Best-effort StopStream first to quiesce it.
            try {
                t.send_cmd(protocol::Cmd::StopStream, {},
                           std::chrono::milliseconds(500));
            } catch (...) { /* expected when fw is already idle */ }

            // 1) GetDevType — firmware-authoritative side (LEFT=0/RIGHT=1/
            //    UNKNOWN=0xFF). Primary source; survives firmware too old for
            //    GetSn.
            try {
                auto dt = t.send_cmd(protocol::Cmd::GetDevType, {},
                                     std::chrono::milliseconds(500));
                if (!dt.is_nack && dt.data.size() == 1) {
                    if (dt.data[0] == static_cast<uint8_t>(protocol::DeviceType::Left))
                        dev_side = Side::Left;
                    else if (dt.data[0] == static_cast<uint8_t>(protocol::DeviceType::Right))
                        dev_side = Side::Right;
                }
            } catch (...) { /* very old fw without dev-type — leave unset */ }

            // 2) GetSn — full TacCap SN, the authoritative source of BOTH the
            //    side (sequence parity) and the leader/follower role (patch
            //    suffix). Retry a few times: right after a cold plug-in the
            //    USB-CDC link and firmware are still settling, so the first
            //    command(s) can be dropped and a single 500ms attempt misses
            //    the SN — which would otherwise leave the side Unknown on the
            //    very first scan. May still time out on firmware that predates
            //    SN support; that's fine, side/role just stay Unknown.
            for (int attempt = 0; attempt < 3 && fw_sn.empty(); ++attempt) {
                try {
                    auto ack = t.send_cmd(protocol::Cmd::GetSn, {},
                                          std::chrono::milliseconds(500));
                    if (!ack.is_nack && !ack.data.empty()) {
                        fw_sn = protocol::decode_sn(ack.data.data(), ack.data.size());
                    }
                } catch (...) { /* timed out — retry while the link warms up */ }
            }
        } catch (const IoError& e) {
            // Ports are opened exclusively. EBUSY means a gripper is already
            // open on it -- report that rather than an unidentified board.
            if (e.errno_value() == EBUSY) {
                in_use = true;
            }
        } catch (...) {
            // Any other open/probe failure: side and role stay Unknown.
        }

        const auto parsed = parse_serial(fw_sn);

        GripperEndpoints e{};
        e.side        = parsed.side ? *parsed.side     // firmware SN (authoritative)
                      : dev_side    ? *dev_side        // GetDevType (firmware) fallback
                                    : Side::Unknown;   // no firmware source — never guess
        e.mcu_device  = mcu.device;
        e.mcu_serial  = mcu.serial_number;
        e.firmware_sn = std::move(fw_sn);
        e.role        = parsed.role;
        e.in_use      = in_use;
        out.push_back(std::move(e));
    }
    return out;
}

GripperEndpoints find_one() {
    auto v = scan_all();
    if (v.empty()) {
        throw IoError("discovery::find_one: no gripper detected", ENODEV);
    }
    if (v.size() > 1) {
        throw IoError("discovery::find_one: multiple grippers detected (" +
                      std::to_string(v.size()) +
                      "); use find_left() or find_right()", EINVAL);
    }
    return v.front();
}

GripperEndpoints find_left() {
    for (const auto& g : scan_all()) {
        if (g.side == Side::Left) return g;
    }
    throw IoError("discovery::find_left: no left-side gripper detected "
                  "(side comes from GetDevType / the SN sequence; check the "
                  "device's burned DEV_TYPE)", ENODEV);
}

GripperEndpoints find_right() {
    for (const auto& g : scan_all()) {
        if (g.side == Side::Right) return g;
    }
    throw IoError("discovery::find_right: no right-side gripper detected "
                  "(side comes from GetDevType / the SN sequence; check the "
                  "device's burned DEV_TYPE)", ENODEV);
}

GripperEndpoints find_leader() {
    for (const auto& g : scan_all()) {
        if (g.role == Role::Leader) return g;
    }
    throw IoError("discovery::find_leader: no leader gripper detected "
                  "(firmware SN must end with patch suffix 'm'; legacy or "
                  "unburned SNs report role Unknown)", ENODEV);
}

GripperEndpoints find_follower() {
    for (const auto& g : scan_all()) {
        if (g.role == Role::Follower) return g;
    }
    throw IoError("discovery::find_follower: no follower gripper detected "
                  "(firmware SN must end with patch suffix 's'; legacy or "
                  "unburned SNs report role Unknown)", ENODEV);
}

}  // namespace xense::taccap::discovery
