#pragma once

#include <initializer_list>
#include <cstdint>

#include "lib15442c/device/motor.hpp"

namespace config
{
    constexpr std::initializer_list<std::int8_t> PORT_LEFT_DRIVE = {-16, -18, -17};
    constexpr std::initializer_list<std::int8_t> PORT_RIGHT_DRIVE = {12, 13, 14};

    constexpr double DRIVE_WHEEL_DIAMETER = 3.25;
    constexpr double DRIVE_GEAR_RATIO = 36.0 / 48.0;
    constexpr double DRIVE_TRACK_WIDTH = 14; // TODO: get real number

    constexpr lib15442c::MotorBrakeMode DRIVE_BRAKE_MODE = lib15442c::MotorBrakeMode::COAST;

    constexpr lib15442c::MotorGroupParameters PARAMS_LEFT_DRIVE = {
        reversed : true,
        brake_mode : config::DRIVE_BRAKE_MODE,
        ratio : lib15442c::MOTOR_BLUE *config::DRIVE_GEAR_RATIO
    };

    constexpr lib15442c::MotorGroupParameters PARAMS_RIGHT_DRIVE = {
        reversed : false,
        brake_mode : config::DRIVE_BRAKE_MODE,
        ratio : lib15442c::MOTOR_BLUE *config::DRIVE_GEAR_RATIO
    };

    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
}