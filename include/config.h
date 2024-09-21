#pragma once

#include <initializer_list>
#include <cstdint>
#include "mechanism/intake.h"
#include "mechanism/arm.h"

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

    constexpr double ARM_GEAR_RATIO = 12.0 / 84.0;
    constexpr lib15442c::MotorParameters PARAMS_ARM = {
        port : 6,
        reversed : true,
        brake_mode : lib15442c::MotorBrakeMode::HOLD,
        ratio : lib15442c::MOTOR_GREEN *config::ARM_GEAR_RATIO
    };
    constexpr int PORT_ARM_ROTATION = -10;
    constexpr char PORT_ARM_LIMIT = 'F';
    constexpr lib15442c::PIDParameters PARAMS_ARM_PID = {
        kP : 20.0,
        kI : 0.0,
        kD : 0.0
    };
    constexpr mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : 0.0,
        color_sort : 35.0,
        alliance_stake : 50.0,
        neutral_stake : 93.0
    };


    constexpr double INTAKE_GEAR_RATIO = 24.0 / 16.0;
    constexpr lib15442c::MotorParameters PARAMS_INTAKE = {
        port : 8,
        reversed : false,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_BLUE *config::INTAKE_GEAR_RATIO
    };
    constexpr int PORT_OPTICAL = 12;
    constexpr char PORT_REDIRECT = 'G';


    constexpr char PORT_CLAMP = 'H';

    constexpr int PORT_IMU = 19;
    constexpr double IMU_SCALE = 1.0;

    constexpr int PORT_PARALLEL_TRACKER = 21;
    constexpr int PORT_PERPENDICULAR_TRACKER = 20;
    constexpr double PARALLEL_TRACKER_OFFSET = 0.0;
    constexpr double PERPENDICULAR_TRACKER_OFFSET = 0.0;
    constexpr double PARALLEL_TRACKER_DIAMETER = 2.75;
    constexpr double PERPENDICULAR_TRACKER_DIAMETER = 2.0;

    constexpr double DRIVE_SLEW_RATE = 127.0 / 0.25;
    constexpr double DRIVE_KP = 1.0;
    constexpr double DRIVE_KI = 0.0;
    constexpr double DRIVE_KD = 0.0;
    constexpr double TURN_KP = 20.0;
    constexpr double TURN_KI = 0.0;
    constexpr double TURN_KD = 0.0;

    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
    std::shared_ptr<mechanism::Arm> make_arm();
    std::shared_ptr<mechanism::Intake> make_intake();

    std::shared_ptr<lib15442c::TrackerOdom> make_tracker_odom();
    std::shared_ptr<lib15442c::DriveController> make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry);
}