#pragma once

#include <initializer_list>
#include <cstdint>
#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "mechanism/arm.h"
#include "mechanism/intake.h"

namespace config
{
    constexpr std::initializer_list<int> PORT_LEFT_DRIVE = {13, 12, -15};
    constexpr std::initializer_list<int> PORT_RIGHT_DRIVE = {18, 19, -20};

    constexpr double DRIVE_WHEEL_DIAMETER = 3.25;
    constexpr double DRIVE_GEAR_RATIO = 36.0 / 48.0;
    constexpr double DRIVE_TRACK_WIDTH = 11.5;
    constexpr lib15442c::MotorBrakeMode DRIVE_BRAKE_MODE = lib15442c::MotorBrakeMode::COAST;

    constexpr lib15442c::MotorGroupParameters PARAMS_LEFT_DRIVE = {
        reversed : true,
        brake_mode : config::DRIVE_BRAKE_MODE,
        ratio : lib15442c::MOTOR_BLUE * DRIVE_GEAR_RATIO
    };
    constexpr lib15442c::MotorGroupParameters PARAMS_RIGHT_DRIVE = {
        reversed : false,
        brake_mode : config::DRIVE_BRAKE_MODE,
        ratio : lib15442c::MOTOR_BLUE * DRIVE_GEAR_RATIO
    };

    
    constexpr lib15442c::TrajectoryConstraints TRAJECTORY_CONSTRAINTS = {
        // max_speed: 74.22,
        max_speed: 80.0,
        // starting_acceleration: 300,
        starting_acceleration: 300.0,

        track_width: DRIVE_TRACK_WIDTH
    };
    constexpr lib15442c::FeedforwardConstants FEEDFORWARD_CONSTANTS = {
        // voltage required to overcome static friction
        kS: 20,
        // how much voltage to apply per in/s while maintaining speed
        // kV: 1.4445,
        kV: 1.35153787308,
        // how much voltage to apply per in/s/s of acceleration
        // kA: 0.356666666667,
        // kA_down: 0.356666666667 * 1.0,

        kA: 0.411787878788,
        kA_down: 0.411787878788 * 1.0,
        // how much voltage to apply per in/s of error in velocity
        kP: 2.0
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
        kD : 1.0
    };
    constexpr mechanism::ArmTargetConfig ARM_TARGET_CONFIG = {
        load : -41.0,
        alliance_stake : 10.0,
        ladder_touch : 20.0,
        neutral_stake : 43.0,
        climb : 39.0
    };
    constexpr int PORT_ARM_ROTATION = 4;

    
    constexpr lib15442c::MotorParameters PARAMS_INTAKE = {
        port: 2,
        reversed : true,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_GREEN,
    };
    constexpr int PORT_OPTICAL = 12;
    constexpr char PORT_REDIRECT = 'A';


    constexpr char PORT_CLAMP = 'B';
    constexpr char PORT_DOINKER = 'C';
    constexpr char PORT_INTAKE_LIFT = 'H';
    constexpr char PORT_ALLIANCE_STAKE_ADJUST = 'B';

    // constexpr int PORT_IMU = 7;
    // constexpr double IMU_SCALE = 1.00538559931;
    constexpr int PORT_IMU_2 = 6;
    constexpr double IMU_SCALE_2 = 1.01077119862;

    constexpr int PORT_PARALLEL_TRACKER = 17;
    constexpr int PORT_PERPENDICULAR_TRACKER = 16;
    constexpr double PARALLEL_TRACKER_OFFSET = -106.93905;
    constexpr double PERPENDICULAR_TRACKER_OFFSET = -125.20224;
    // constexpr double PERPENDICULAR_TRACKER_OFFSET_MOGO = -61.11289;
    constexpr double PARALLEL_TRACKER_DIAMETER = 2.75;
    constexpr double PERPENDICULAR_TRACKER_DIAMETER = 2.75;

    constexpr double DRIVE_SLEW_RATE = (127.0 / 0.35) / (20.0/1000.0);
    // constexpr double DRIVE_KP = 11.0;
    constexpr double DRIVE_KP = 6.0;
    // constexpr double DRIVE_KI = 4.0;
    // constexpr double DRIVE_KI_RANGE = 1.0;
    // constexpr double DRIVE_KD = 40.0;
    constexpr double DRIVE_KI = 0.0;
    constexpr double DRIVE_KI_RANGE = 1.0;
    constexpr double DRIVE_KD = 17.0;
    constexpr double TURN_KP = 5.5;
    constexpr double TURN_KI = 3.0;
    constexpr double TURN_KI_RANGE = 5.0;
    constexpr double TURN_KD = 23.0;

    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
    std::shared_ptr<mechanism::Intake> make_intake();
    std::shared_ptr<mechanism::Arm> make_arm(std::shared_ptr<mechanism::Intake> intake);

    std::shared_ptr<lib15442c::TrackerOdom> make_tracker_odom();
    std::shared_ptr<lib15442c::DriveController> make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry);
}