#pragma once

#include <initializer_list>
#include <cstdint>
#include "mechanism/ring_mech.h"

namespace config
{
    constexpr std::initializer_list<int> PORT_LEFT_DRIVE = {-15, -14, -13};
    constexpr std::initializer_list<int> PORT_RIGHT_DRIVE = {5, 4, 3};

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

    constexpr lib15442c::MotorGroupParameters PARAMS_RING_MECH = {
        reversed : false,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_BLUE,
    };
    constexpr std::initializer_list<int> PORT_RING_MECH = {6, -8};
    constexpr lib15442c::PIDParameters PARAMS_ARM_PID = {
        kP : 10.0,
        kI : 0.0,
        kD : 0.0
    };
    constexpr mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : 0.0,
        alliance_stake : 30.0,
        ladder_touch : 60.0,
        neutral_stake : 65.0
    };
    constexpr int PORT_ARM_ROTATION = 10;
    constexpr char PORT_ARM_LIMIT = 'F';
    constexpr int PORT_OPTICAL = 12;
    constexpr char PORT_REDIRECT = 'G';


    constexpr char PORT_CLAMP = 'H';
    constexpr char PORT_OINKER = 'E';
    constexpr char PORT_ALLIANCE_STAKE_ADJUST = 'B';

    constexpr int PORT_IMU = 7;
    constexpr double IMU_SCALE = 1.00287313022;

    constexpr int PORT_PARALLEL_TRACKER = 2;
    constexpr int PORT_PERPENDICULAR_TRACKER = 11;
    constexpr double PARALLEL_TRACKER_OFFSET = -0.274026;
    constexpr double PERPENDICULAR_TRACKER_OFFSET = 1.73918;
    constexpr double PARALLEL_TRACKER_DIAMETER = 2.75;
    constexpr double PERPENDICULAR_TRACKER_DIAMETER = 2.0;

    constexpr double DRIVE_SLEW_RATE = (127.0 / 0.35) / (20.0/1000.0);
    constexpr double DRIVE_KP = 13.0;
    constexpr double DRIVE_KI = 0.0;
    constexpr double DRIVE_KD = 50.0;
    constexpr double TURN_KP = 5.5;
    constexpr double TURN_KI = 0.0;
    constexpr double TURN_KD = 30.0;

    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
    std::shared_ptr<mechanism::RingMech> make_ring_mech();

    std::shared_ptr<lib15442c::TrackerOdom> make_tracker_odom();
    std::shared_ptr<lib15442c::DriveController> make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry);
}