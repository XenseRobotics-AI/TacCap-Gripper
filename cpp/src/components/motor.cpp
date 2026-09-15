// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/components/motor.hpp>
#include <taccap/error.hpp>
#include <taccap/protocol/codec.hpp>

#include <cstring>
#include <cmath>
#include <stdexcept>

namespace xense::taccap {

namespace {

void send_or_throw(bus::Transport& t, protocol::Cmd cmd,
                   const std::vector<uint8_t>& payload, const char* what) {
    auto ack = t.send_cmd(cmd, payload);
    if (bus::ack_error_code(ack) != protocol::ErrorCode::Ok) {
        throw ProtocolError(std::string("Motor::") + what + " NACK: " +
                            protocol::to_string(bus::ack_error_code(ack)));
    }
}


bool magnitude(float x, float limit) { return std::isfinite(x) && std::abs(x) <= limit; }
bool positive(float x, float limit) { return magnitude(x, limit) && x > 0; }
void validate(const protocol::MotorPosCtrl& c) {
    if (!magnitude(c.target_pos, 12.57f) || !positive(c.max_vel, 50) || !positive(c.max_torque, 5.5f))
        throw std::invalid_argument("invalid RS05 position target or limits");
}
void validate(const protocol::MotorVelCtrl& c) {
    if (!magnitude(c.target_vel, 50) || !positive(c.max_torque, 5.5f) || !positive(c.profile_acc, 1000))
        throw std::invalid_argument("invalid RS05 velocity target or limits");
}
void validate(const protocol::MotorTorqueCtrl& c) {
    if (!magnitude(c.target_torque, 5.5f) || !positive(c.max_vel, 50))
        throw std::invalid_argument("invalid RS05 torque target or limits");
}
void validate(const protocol::MotorImpedanceCtrl& c) {
    if (!magnitude(c.target_pos, 12.57f) || !magnitude(c.vel, 50) ||
        !magnitude(c.target_torque, 5.5f) || !magnitude(c.kp, 500) || c.kp < 0 ||
        !magnitude(c.kd, 5) || c.kd < 0)
        throw std::invalid_argument("invalid RS05 impedance target or gains");
}

}  // namespace

Motor::Motor(bus::Transport& transport) : t_(transport) {}

MotorStatusSample Motor::decode(const std::uint8_t* payload, std::size_t len) {
    MotorStatusSample s{};
    s.host_time      = std::chrono::steady_clock::now();
    s.raw            = protocol::decode_motor_status(payload, len);
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
    constexpr size_t extended_size = sizeof(protocol::MotorStatus) +
        sizeof(protocol::MotorExecutionStatus) + sizeof(uint32_t) + sizeof(float);
    static_assert(extended_size == 107);
    if (len == extended_size) {
        s.has_execution = true;
        std::memcpy(&s.execution, payload + sizeof(protocol::MotorStatus), sizeof(s.execution));
        std::memcpy(&s.sample_age_ms, payload + extended_size - sizeof(float) - sizeof(uint32_t), sizeof(uint32_t));
        std::memcpy(&s.actual_pos, payload + extended_size - sizeof(float), sizeof(float));
    }
    return s;
}

void Motor::enable()       { auto lock = check_control_access(); send_or_throw(t_, protocol::Cmd::MotorEnable,     {}, "enable"); }
void Motor::disable() {
    // Serialize behind any in-flight target, then prevent this controller from
    // re-enabling the motor on its next tick. stop()/start() acquires a new lease.
    std::lock_guard<std::mutex> lock(control_mu_);
    if (controller_) controller_disabled_ = true;
    send_or_throw(t_, protocol::Cmd::MotorDisable, {}, "disable");
}
void Motor::clear_fault()  { auto lock = check_control_access(); send_or_throw(t_, protocol::Cmd::MotorClearFault, {}, "clear_fault"); }

void Motor::set_position(float pos, float max_vel, float max_torque) {
    auto control_lock = check_control_access();
    protocol::MotorPosCtrl c{pos, max_vel, max_torque};
    validate(c);
    send_or_throw(t_, protocol::Cmd::MotorPosCtrl, protocol::encode(c), "set_position");
}

void Motor::set_velocity(float vel, float max_torque, float profile_acc) {
    auto control_lock = check_control_access();
    protocol::MotorVelCtrl c{vel, max_torque, profile_acc};
    validate(c);
    send_or_throw(t_, protocol::Cmd::MotorVelCtrl, protocol::encode(c), "set_velocity");
}

void Motor::set_torque(float torque, float max_vel) {
    auto control_lock = check_control_access();
    protocol::MotorTorqueCtrl c{torque, max_vel, 0.0f};
    validate(c);
    send_or_throw(t_, protocol::Cmd::MotorTorqueCtrl, protocol::encode(c), "set_torque");
}

void Motor::set_impedance(float pos, float kp, float kd, float ff_torque,
                          float ff_vel) {
    auto control_lock = check_control_access();
    protocol::MotorImpedanceCtrl c{pos, kp, kd, ff_torque, ff_vel};
    validate(c);
    send_or_throw(t_, protocol::Cmd::MotorImpedanceCtrl, protocol::encode(c), "set_impedance");
}

// ---- High-rate control submission (no ACK) --------------------------------
// Fire-and-forget via Transport::send_cmd_no_ack — no ACK wait, no retry, no
// throw on NACK (there is none). The struct overloads are the single source of
// truth; the float wrappers delegate. See motor.hpp for the health contract.
void Motor::submit(const protocol::MotorImpedanceCtrl& c) {
    auto control_lock = check_control_access();
    validate(c);
    t_.send_cmd_no_ack(protocol::Cmd::MotorImpedanceCtrl, protocol::encode(c));
}
void Motor::submit(const protocol::MotorPosCtrl& c) {
    auto control_lock = check_control_access();
    validate(c);
    t_.send_cmd_no_ack(protocol::Cmd::MotorPosCtrl, protocol::encode(c));
}
void Motor::submit(const protocol::MotorVelCtrl& c) {
    auto control_lock = check_control_access();
    validate(c);
    t_.send_cmd_no_ack(protocol::Cmd::MotorVelCtrl, protocol::encode(c));
}
void Motor::submit(const protocol::MotorTorqueCtrl& c) {
    auto control_lock = check_control_access();
    validate(c);
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

}  // namespace xense::taccap

namespace xense::taccap {
protocol::MotorExecutionStatus Motor::execution_status(std::chrono::milliseconds timeout) {
    auto ack = t_.send_cmd(protocol::Cmd::GetMotorExecutionStatus, {}, timeout);
    if (ack.is_nack || ack.data.size() != sizeof(protocol::MotorExecutionStatus))
        throw ProtocolError("Motor execution status unavailable (requires firmware 1.1.7)");
    protocol::MotorExecutionStatus out{};
    std::memcpy(&out, ack.data.data(), sizeof(out));
    return out;
}
}

namespace xense::taccap {
void Motor::claim_controller(const void *owner) {
    std::lock_guard<std::mutex> lock(control_mu_);
    if (controller_) throw std::logic_error("motor already owned by a controller");
    controller_ = owner;
    controller_disabled_ = false;
    controller_thread_ = {};
}
void Motor::controller_thread(const void *owner) {
    std::lock_guard<std::mutex> lock(control_mu_);
    if (controller_ != owner) throw std::logic_error("invalid motor controller owner");
    controller_thread_ = std::this_thread::get_id();
}
void Motor::release_controller(const void *owner) {
    std::lock_guard<std::mutex> lock(control_mu_);
    if (controller_ == owner) { controller_ = nullptr; controller_thread_ = {}; }
}
std::unique_lock<std::mutex> Motor::check_control_access() {
    std::unique_lock<std::mutex> lock(control_mu_);
    if (controller_ && (controller_disabled_ || controller_thread_ != std::this_thread::get_id()))
        throw std::logic_error("motor targets are owned by the active controller");
    return lock;
}
}
