#pragma once

#include <initializer_list>
#include <cstdint>
#include "mechanism/arm.h"
#include "mechanism/intake.h"

namespace config
{
    constexpr std::initializer_list<int> PORT_LEFT_DRIVE = {13, 12, -15};
    constexpr std::initializer_list<int> PORT_RIGHT_DRIVE = {18, 19, -20};

    constexpr double DRIVE_WHEEL_DIAMETER = 3.25;
    constexpr double DRIVE_GEAR_RATIO = 36.0 / 48.0;
    constexpr double DRIVE_TRACK_WIDTH = 11.5; // TODO: get real number
    constexpr lib15442c::MotorBrakeMode DRIVE_BRAKE_MODE = lib15442c::MotorBrakeMode::COAST;

    constexpr lib15442c::MotorGroupParameters PARAMS_LEFT_DRIVE = {
        reversed : true,
        brake_mode : config::DRIVE_BRAKE_MODE,
        ratio : lib15442c::MOTOR_BLUE * config::DRIVE_GEAR_RATIO
    };

    constexpr lib15442c::MotorGroupParameters PARAMS_RIGHT_DRIVE = {
        reversed : false,
        brake_mode : config::DRIVE_BRAKE_MODE,
        ratio : lib15442c::MOTOR_BLUE * DRIVE_GEAR_RATIO
    };

    constexpr lib15442c::DrivetrainConstraints DRIVE_CONSTRAINTS = {
        max_speed: 76,
        max_acceleration: 76 * 1.7,

        track_width: DRIVE_TRACK_WIDTH
    };

    constexpr lib15442c::MotorGroupParameters PARAMS_ARM = {
        reversed : false,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_GREEN,
    };
    constexpr std::initializer_list<int> PORT_ARM = {1, -10};
    constexpr lib15442c::PIDParameters PARAMS_ARM_PID = {
        kP : 5.0,
        kI : 0.0,
        kD : 0.0
    };
    constexpr mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : 0.0,
        alliance_stake : 40.0,
        ladder_touch : 40.0,
        neutral_stake : 55.0
    };
    constexpr int PORT_ARM_ROTATION = 4;

    
    constexpr lib15442c::MotorParameters PARAMS_INTAKE = {
        port: 2,
        reversed : false,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_GREEN,
    };
    constexpr int PORT_OPTICAL = 12;
    constexpr char PORT_REDIRECT = 'A';


    constexpr char PORT_CLAMP = 'B';
    constexpr char PORT_OINKER = 'E';
    constexpr char PORT_ALLIANCE_STAKE_ADJUST = 'B';

    constexpr int PORT_IMU = 3;
    constexpr double IMU_SCALE = 1.00287313022;

    constexpr int PORT_PARALLEL_TRACKER = 17;
    constexpr int PORT_PERPENDICULAR_TRACKER = 14;
    constexpr double PARALLEL_TRACKER_OFFSET = 0;
    constexpr double PERPENDICULAR_TRACKER_OFFSET = 0;
    constexpr double PARALLEL_TRACKER_DIAMETER = 2.75;
    constexpr double PERPENDICULAR_TRACKER_DIAMETER = 2.75;

    constexpr double DRIVE_SLEW_RATE = (127.0 / 0.35) / (20.0/1000.0);
    constexpr double DRIVE_KP = 13.0;
    constexpr double DRIVE_KI = 0.0;
    constexpr double DRIVE_KD = 50.0;
    constexpr double TURN_KP = 5.5;
    constexpr double TURN_KI = 0.0;
    constexpr double TURN_KD = 30.0;

    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
    std::shared_ptr<mechanism::Intake> make_intake();
    std::shared_ptr<mechanism::Arm> make_arm();

    std::shared_ptr<lib15442c::TrackerOdom> make_tracker_odom();
    std::shared_ptr<lib15442c::DriveController> make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry);
}