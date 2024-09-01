#pragma once

#include <initializer_list>
#include <cstdint>

namespace config
{
    constexpr std::initializer_list<std::int8_t> PORT_LEFT_DRIVE = {-15, -14, -13};
    constexpr std::initializer_list<std::int8_t> PORT_RIGHT_DRIVE = {5, 4, 3};

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


    constexpr double ARM_GEAR_RATIO = 36.0 / 84.0;
    constexpr lib15442c::MotorParameters PARAMS_ARM = {
        port : 6,
        reversed: true,
        brake_mode : lib15442c::MotorBrakeMode::BRAKE,
        ratio : lib15442c::MOTOR_RED * config::ARM_GEAR_RATIO
    };


    constexpr double INTAKE_GEAR_RATIO = 24.0 / 16.0;
    constexpr lib15442c::MotorParameters PARAMS_INTAKE = {
        port : 8,
        reversed: false,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_BLUE * config::INTAKE_GEAR_RATIO
    };


    constexpr int PORT_OPTICAL = 12;
    constexpr char PORT_CLAMP = 'H';
    constexpr char PORT_REDIRECT = 'G';


    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
    lib15442c::Motor make_arm();
    lib15442c::Motor make_intake();
}