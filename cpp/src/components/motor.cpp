// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/motor.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cstring>
#include <string>

namespace xense::taccap {

namespace {

void send_or_throw(bus::Transport& t, protocol::Cmd cmd,
                   const std::vector<uint8_t>& payload, const char* what) {
    auto ack = t.send_cmd(cmd, payload);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::") + what + " NACK: " +
                            protocol::to_string(ack.error_code));
    }
}

}  // namespace

Motor::Motor(bus::Transport& transport) : t_(transport) {}

MotorStatusSample Motor::decode(const std::uint8_t* payload, std::size_t len) {
    MotorStatusSample s{};
    s.host_time      = std::chrono::steady_clock::now();
    // Progressive: accepts 31 / 59 / 72 and zero-fills the tail, so this one
    // decoder serves both the legacy prefix and the V2 stream.
    s.raw            = protocol::decode_motor_status_ext(payload, len);
    s.actual_pos     = s.raw.actual_pos;
    s.actual_vel     = s.raw.actual_vel;
    s.actual_torque  = s.raw.actual_torque;
    s.motor_temp_c   = s.raw.motor_temp;
    s.status         = s.raw.status;
    // target_* / control_mode: correct on V1.9 (31B) firmware.
    s.target_pos     = s.raw.target_pos;
    s.target_vel     = s.raw.target_vel;
    s.target_torque  = s.raw.target_torque;
    s.control_mode   = s.raw.control_mode;
    s.monitor_version    = s.raw.monitor_version;
    s.stop_reason        = s.raw.stop_reason;
    s.monitor_reserved   = s.raw.monitor_reserved;
    s.fault_code         = s.raw.fault_code;
    s.latched_fault_code = s.raw.latched_fault_code;
    return s;
}

void Motor::enable()       { send_or_throw(t_, protocol::Cmd::MotorEnable,     {}, "enable"); }
void Motor::disable()      { send_or_throw(t_, protocol::Cmd::MotorDisable,    {}, "disable"); }
void Motor::clear_fault()  { send_or_throw(t_, protocol::Cmd::MotorClearFault, {}, "clear_fault"); }

void Motor::set_position(float pos, float max_vel, float max_torque) {
    protocol::MotorPosCtrl c{pos, max_vel, max_torque};
    send_or_throw(t_, protocol::Cmd::MotorPosCtrl, protocol::encode(c), "set_position");
}

void Motor::set_velocity(float vel, float max_torque, float profile_acc) {
    protocol::MotorVelCtrl c{vel, max_torque, profile_acc};
    send_or_throw(t_, protocol::Cmd::MotorVelCtrl, protocol::encode(c), "set_velocity");
}

void Motor::set_torque(float torque, float max_vel) {
    protocol::MotorTorqueCtrl c{torque, max_vel, 0.0f};
    send_or_throw(t_, protocol::Cmd::MotorTorqueCtrl, protocol::encode(c), "set_torque");
}

void Motor::set_impedance(float pos, float kp, float kd, float ff_torque,
                          float ff_vel) {
    protocol::MotorImpedanceCtrl c{pos, kp, kd, ff_torque, ff_vel};
    send_or_throw(t_, protocol::Cmd::MotorImpedanceCtrl, protocol::encode(c), "set_impedance");
}

// ---- High-rate control submission (no ACK) --------------------------------
// Fire-and-forget via Transport::send_cmd_no_ack — no ACK wait, no retry, no
// throw on NACK (there is none). The struct overloads are the single source of
// truth; the float wrappers delegate. See motor.hpp for the health contract.
void Motor::submit(const protocol::MotorImpedanceCtrl& c) {
    t_.send_cmd_no_ack(protocol::Cmd::MotorImpedanceCtrl, protocol::encode(c));
}
void Motor::submit(const protocol::MotorPosCtrl& c) {
    t_.send_cmd_no_ack(protocol::Cmd::MotorPosCtrl, protocol::encode(c));
}
void Motor::submit(const protocol::MotorVelCtrl& c) {
    t_.send_cmd_no_ack(protocol::Cmd::MotorVelCtrl, protocol::encode(c));
}
void Motor::submit(const protocol::MotorTorqueCtrl& c) {
    t_.send_cmd_no_ack(protocol::Cmd::MotorTorqueCtrl, protocol::encode(c));
}

void Motor::submit_impedance(float pos, float kp, float kd, float ff_torque,
                             float ff_vel) {
    submit(protocol::MotorImpedanceCtrl{pos, kp, kd, ff_torque, ff_vel});
}
void Motor::submit_position(float pos, float max_vel, float max_torque) {
    submit(protocol::MotorPosCtrl{pos, max_vel, max_torque});
}
void Motor::submit_velocity(float vel, float max_torque, float profile_acc) {
    submit(protocol::MotorVelCtrl{vel, max_torque, profile_acc});
}
void Motor::submit_torque(float torque, float max_vel) {
    submit(protocol::MotorTorqueCtrl{torque, max_vel, 0.0f});
}

MotorStatusSample Motor::read_status(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorStatus, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::read_status NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return decode(ack.data.data(), ack.data.size());
}

protocol::MotorStatusExt Motor::read_status_ext(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorStatusExt, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::read_status_ext NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_motor_status_ext(ack.data.data(), ack.data.size());
}

protocol::MotorFaultReport Motor::fault_report(bool force,
                                               std::chrono::milliseconds timeout) {
    // 0 bytes = read the cache; 1 non-zero byte = force a CAN round trip. The
    // firmware accepts either length, so send the shorter frame when we can.
    std::vector<uint8_t> req;
    if (force) req.push_back(1);
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorFault, req, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::fault_report NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_motor_fault_report(ack.data.data(), ack.data.size());
}

// A firmware that does not have 0x58 answers InvalidCmd, which is the single
// most likely reason this call fails — say so instead of leaving the caller
// with a bare "NACK: InvalidCmd" that names no remedy.
static std::string motor_version_hint(protocol::ErrorCode err) {
    if (err != protocol::ErrorCode::InvalidCmd) return {};
    return " — GetMotorVersion (0x58) needs follower firmware >= 1.2.6; an "
           "older follower does not have the command at all";
}

protocol::MotorVersion Motor::motor_version(std::chrono::milliseconds timeout) {
    // A NACK reaches us two different ways and the obvious one is the path
    // this code does NOT take: the transport turns the pure-ACK error shape
    // into an exception inside send_cmd, so an `if (ack.is_nack)` check after
    // it is dead code. That is exactly how the firmware-version hint below
    // came to never fire despite being written down — measured on a 1.2.5
    // follower, the caller got a bare "NACK: InvalidCmd".
    bus::AckResponse ack;
    try {
        ack = t_.send_cmd(protocol::Cmd::GetMotorVersion, {}, timeout);
    } catch (const ProtocolError& e) {
        const std::string invalid =
            protocol::to_string(protocol::ErrorCode::InvalidCmd);
        const std::string what = e.what();
        if (what.find(invalid) != std::string::npos) {
            throw ProtocolError(
                what + motor_version_hint(protocol::ErrorCode::InvalidCmd));
        }
        throw;
    }
    // The second shape: the firmware echoed the command and packed the error
    // into a single payload byte. That one does NOT throw, and without this it
    // would fall through to the size check below and be misreported as a short
    // payload. bus::ack_error_code resolves both shapes.
    if (const auto err = bus::ack_error_code(ack);
        err != protocol::ErrorCode::Ok) {
        throw ProtocolError(std::string("Motor::motor_version NACK: ") +
                            protocol::to_string(err) +
                            motor_version_hint(err));
    }
    if (ack.data.size() < protocol::MOTOR_VERSION_SIZE) {
        throw ProtocolError("Motor::motor_version: short payload");
    }
    protocol::MotorVersion out{};
    std::memcpy(&out, ack.data.data(), protocol::MOTOR_VERSION_SIZE);
    return out;
}

Motor::SubId Motor::on_status(Callback cb) {
    return t_.subscribe(
        protocol::Cmd::GetMotorStatus,
        [cb = std::move(cb)](const bus::Frame& f) {
            try {
                cb(decode(f.payload.data(), f.payload.size()));
            } catch (...) {}
        });
}

void Motor::off(SubId id) { t_.unsubscribe(id); }

// ---- Follower motor admin (follower-only; validated against hw_v1.1.0) -----

void Motor::set_zero() {
    send_or_throw(t_, protocol::Cmd::MotorSetZero, {}, "set_zero");
}

uint8_t Motor::get_can_id() {
    auto ack = t_.send_cmd(protocol::Cmd::MotorGetCanId, {});
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::get_can_id NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    if (ack.data.empty()) {
        throw ProtocolError("Motor::get_can_id: empty response");
    }
    return ack.data[0];
}

void Motor::set_can_id(uint8_t can_id) {
    send_or_throw(t_, protocol::Cmd::MotorSetCanId, {can_id}, "set_can_id");
}

void Motor::switch_protocol(protocol::MotorProtocol p) {
    send_or_throw(t_, protocol::Cmd::MotorSwitchProtocol,
                  {static_cast<uint8_t>(p)}, "switch_protocol");
}

protocol::MotorProtocol Motor::get_protocol() {
    auto ack = t_.send_cmd(protocol::Cmd::MotorGetProtocol, {});
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::get_protocol NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    if (ack.data.empty()) {
        throw ProtocolError("Motor::get_protocol: empty response");
    }
    return static_cast<protocol::MotorProtocol>(ack.data[0]);
}

protocol::MotorPrivateParam Motor::get_private_param(uint16_t index) {
    const std::vector<uint8_t> req{static_cast<uint8_t>(index & 0xFF),
                                   static_cast<uint8_t>((index >> 8) & 0xFF)};
    auto ack = t_.send_cmd(protocol::Cmd::MotorGetPrivateParam, req);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::get_private_param NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    // On rejection (e.g. under MIT, or a non-whitelisted index) the firmware
    // answers with a 1-byte error code instead of the 8-byte param; a
    // successful read is the full 8 bytes.
    if (ack.data.size() == 1) {
        throw ProtocolError(std::string("Motor::get_private_param rejected: ") +
            protocol::to_string(static_cast<protocol::ErrorCode>(ack.data[0])));
    }
    return protocol::decode_motor_private_param(ack.data.data(), ack.data.size());
}

void Motor::set_private_param(uint16_t index, uint32_t raw_value) {
    std::vector<uint8_t> req(6);
    req[0] = static_cast<uint8_t>(index & 0xFF);
    req[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
    std::memcpy(req.data() + 2, &raw_value, 4);  // little-endian host == wire
    send_or_throw(t_, protocol::Cmd::MotorSetPrivateParam, req, "set_private_param");
}

protocol::MotorControlStats Motor::control_stats(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorControlStats, {}, timeout);
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::control_stats NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    return protocol::decode_motor_control_stats(ack.data.data(), ack.data.size());
}

void Motor::set_startup_limit_torque(float torque_nm) {
    std::vector<uint8_t> req(4);
    std::memcpy(req.data(), &torque_nm, 4);  // little-endian host == wire
    send_or_throw(t_, protocol::Cmd::MotorSetStartupLimitTorque, req,
                  "set_startup_limit_torque");
}

float Motor::get_startup_limit_torque() {
    auto ack = t_.send_cmd(protocol::Cmd::MotorGetStartupLimitTorque, {});
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::get_startup_limit_torque NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    if (ack.data.size() < 4) {
        throw ProtocolError("Motor::get_startup_limit_torque: expected 4 bytes, got " +
                            std::to_string(ack.data.size()));
    }
    float torque_nm = 0.0f;
    std::memcpy(&torque_nm, ack.data.data(), 4);
    return torque_nm;
}

protocol::MotorSpec Motor::get_spec() {
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorSpec, {});
    if (ack.is_nack) {
        throw ProtocolError(std::string("Motor::get_spec NACK: ") +
                            protocol::to_string(ack.error_code));
    }
    if (ack.data.size() < sizeof(protocol::MotorSpec)) {
        throw ProtocolError("Motor::get_spec: short payload (" +
                            std::to_string(ack.data.size()) + " bytes) -- "
                            "firmware older than 1.1.6.26 has no 0x56");
    }
    protocol::MotorSpec out{};
    std::memcpy(&out, ack.data.data(), sizeof(out));
    return out;
}

}  // namespace xense::taccap
