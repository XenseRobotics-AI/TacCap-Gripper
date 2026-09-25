// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/protocol/commands.hpp>

namespace xense::taccap::protocol {

const char* to_string(Address a) noexcept {
    switch (a) {
        case Address::PC:        return "PC";
        case Address::MCU:       return "MCU";
        case Address::MCU_RIGHT: return "MCU_RIGHT";
    }
    return "Address?";
}

const char* to_string(FrameType t) noexcept {
    switch (t) {
        case FrameType::CMD_NEED_ACK: return "CMD_NEED_ACK";
        case FrameType::CMD_NO_ACK:   return "CMD_NO_ACK";
        case FrameType::ACK:          return "ACK";
        case FrameType::DATA:         return "DATA";
    }
    return "FrameType?";
}

const char* to_string(Cmd c) noexcept {
    switch (c) {
        case Cmd::Heartbeat:           return "Heartbeat";
        case Cmd::GetVersion:          return "GetVersion";
        case Cmd::ResetDevice:         return "ResetDevice";
        case Cmd::GetSn:               return "GetSn";
        case Cmd::SetSn:               return "SetSn";
        case Cmd::GetDevType:          return "GetDevType";
        case Cmd::SetDevType:          return "SetDevType";
        case Cmd::Ws2812Set:           return "Ws2812Set";
        case Cmd::Ws2812Effect:        return "Ws2812Effect";

        case Cmd::GetImu:              return "GetImu";
        case Cmd::GetEncoder:          return "GetEncoder";
        case Cmd::GetEskin1:           return "GetEskin1";
        case Cmd::GetEskin2:           return "GetEskin2";
        case Cmd::GetAllSensors:       return "GetAllSensors";
        case Cmd::KeyStatus:           return "KeyStatus";

        case Cmd::StartStream:         return "StartStream";
        case Cmd::StopStream:          return "StopStream";
        case Cmd::SetStreamRate:       return "SetStreamRate";
        case Cmd::SetStreamMode:       return "SetStreamMode";
        case Cmd::SetEncoderZero:      return "SetEncoderZero";
        case Cmd::SetImuCal:           return "SetImuCal";
        case Cmd::SetImuMagCal:        return "SetImuMagCal";
        case Cmd::SetCalResult:        return "SetCalResult";
        case Cmd::SetAllCalResult:     return "SetAllCalResult";
        case Cmd::GetCalResult:        return "GetCalResult";
        case Cmd::SensorErrorReport:   return "SensorErrorReport";
        case Cmd::CameraFisheyeCal:    return "CameraFisheyeCal";
        case Cmd::EncoderMaxCal:       return "EncoderMaxCal";

        case Cmd::MotorEnable:         return "MotorEnable";
        case Cmd::MotorDisable:        return "MotorDisable";
        case Cmd::MotorClearFault:     return "MotorClearFault";
        case Cmd::MotorSetZero:        return "MotorSetZero";
        case Cmd::MotorGetCanId:       return "MotorGetCanId";
        case Cmd::MotorSetCanId:       return "MotorSetCanId";
        case Cmd::MotorSwitchProtocol: return "MotorSwitchProtocol";
        case Cmd::MotorGetProtocol:    return "MotorGetProtocol";
        case Cmd::MotorGetPrivateParam: return "MotorGetPrivateParam";
        case Cmd::MotorSetPrivateParam: return "MotorSetPrivateParam";
        case Cmd::MotorSetStartupLimitTorque: return "MotorSetStartupLimitTorque";
        case Cmd::MotorGetStartupLimitTorque: return "MotorGetStartupLimitTorque";
        case Cmd::MotorPosCtrl:        return "MotorPosCtrl";
        case Cmd::MotorVelCtrl:        return "MotorVelCtrl";
        case Cmd::MotorTorqueCtrl:     return "MotorTorqueCtrl";
        case Cmd::MotorImpedanceCtrl:  return "MotorImpedanceCtrl";
        case Cmd::GetMotorStatus:      return "GetMotorStatus";
        case Cmd::GetMotorControlStats: return "GetMotorControlStats";
        case Cmd::GetMotorFault:       return "GetMotorFault";
        case Cmd::GetMotorStatusExt:   return "GetMotorStatusExt";
        case Cmd::GetMotorSpec:        return "GetMotorSpec";
        case Cmd::GetHomeDiag:         return "GetHomeDiag";
        case Cmd::GetMotorVersion:     return "GetMotorVersion";
        case Cmd::GetMotorModel:       return "GetMotorModel";
        case Cmd::SetMotorModel:       return "SetMotorModel";
        case Cmd::MotorCanExtXfer:     return "MotorCanExtXfer";
        case Cmd::GetUartStats:        return "GetUartStats";
        case Cmd::SetLogConfig:        return "SetLogConfig";

        case Cmd::SetImuConfig:        return "SetImuConfig";
        case Cmd::GetImuConfig:        return "GetImuConfig";
        case Cmd::SetEncoderConfig:    return "SetEncoderConfig";
        case Cmd::GetEncoderConfig:    return "GetEncoderConfig";
        case Cmd::SetEskinConfig:      return "SetEskinConfig";
        case Cmd::GetEskinConfig:      return "GetEskinConfig";
        case Cmd::SetGripperConfig:    return "SetGripperConfig";
        case Cmd::GetGripperConfig:    return "GetGripperConfig";
        case Cmd::SetGripperAutoCalConfig: return "SetGripperAutoCalConfig";
        case Cmd::GetGripperAutoCalConfig: return "GetGripperAutoCalConfig";

        case Cmd::OtaStart:            return "OtaStart";
        case Cmd::OtaWriteBlock:       return "OtaWriteBlock";
        case Cmd::OtaVerify:           return "OtaVerify";
        case Cmd::OtaApply:            return "OtaApply";
        case Cmd::OtaAbort:            return "OtaAbort";
        case Cmd::OtaGetStatus:        return "OtaGetStatus";
    }
    return "Cmd?";
}

const char* to_string(ErrorCode e) noexcept {
    switch (e) {
        case ErrorCode::Ok:             return "Ok";
        case ErrorCode::InvalidCmd:     return "InvalidCmd";
        case ErrorCode::InvalidParam:   return "InvalidParam";
        case ErrorCode::LengthMismatch: return "LengthMismatch";
        case ErrorCode::CrcError:       return "CrcError";
        case ErrorCode::Timeout:        return "Timeout";
        case ErrorCode::MotorFault:     return "MotorFault";
        case ErrorCode::SensorOffline:  return "SensorOffline";
        case ErrorCode::SysBusy:        return "SysBusy";
        case ErrorCode::SeqMismatch:    return "SeqMismatch";
        case ErrorCode::CalNotSet:      return "CalNotSet";
        case ErrorCode::OtaBusy:        return "OtaBusy";
        case ErrorCode::OtaNotStarted:  return "OtaNotStarted";
        case ErrorCode::OtaOffsetErr:   return "OtaOffsetErr";
        case ErrorCode::OtaFlashErr:    return "OtaFlashErr";
        case ErrorCode::OtaVerifyFail:  return "OtaVerifyFail";
        case ErrorCode::OtaSizeExceed:  return "OtaSizeExceed";
    }
    return "ErrorCode?";
}

}  // namespace xense::taccap::protocol
