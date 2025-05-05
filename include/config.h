#pragma once

#include <initializer_list>
#include <cstdint>
#include "lib15442c/api.hpp"
#include "mechanism/arm.hpp"
#include "mechanism/ring_manager.hpp"

namespace config
{
    constexpr std::initializer_list<int> PORT_LEFT_DRIVE = {20, 15, -11};
    constexpr std::initializer_list<int> PORT_RIGHT_DRIVE = {10, 13, -12};

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
        // max_speed: 80.0,
        max_speed: 40,
        // starting_acceleration: 300,
        starting_acceleration: 300.0,

        track_width: DRIVE_TRACK_WIDTH
    };
    constexpr lib15442c::FeedforwardConstants FEEDFORWARD_CONSTANTS = {
        // voltage required to overcome static friction
        kS: 20,
        // how much voltage to apply per in/s while maintaining speed
        // kV: 1.4445,
        kV: 1.3,
        // how much voltage to apply per in/s/s of acceleration
        // kA: 0.3566666666,
        // kA_down: 0.3566666666,
        kA: 0.0,
        kA_down: 0.0,
        // how much voltage to apply per in/s of error in velocity
        kP: 0.0
    };

    constexpr lib15442c::MotorGroupParameters PARAMS_LB = {
        reversed : false,
        brake_mode : lib15442c::MotorBrakeMode::HOLD,
        ratio : lib15442c::MOTOR_GREEN,
    };
    constexpr lib15442c::PIDParameters PARAMS_LB_PID = {
        kP: 2,
        kD: 3,
    };
    constexpr std::initializer_list<int> PORT_LB = {-19, 17};
    constexpr int PORT_LB_ROTATION = 18;

    
    constexpr lib15442c::MotorParameters PARAMS_INTAKE = {
        port: 3,
        reversed : true,
        brake_mode : lib15442c::MotorBrakeMode::COAST,
        ratio : lib15442c::MOTOR_GREEN,
    };
    constexpr int PORT_OPTICAL = 5;

    constexpr char PORT_CLAMP = 'F';
    constexpr char PORT_DESCORE = 'E';
    constexpr char PORT_DOINKER = 'C';
    constexpr char PORT_PTO = 'D';
    constexpr char PORT_INTAKE_LIFT = 'A';
    constexpr char PORT_LB_PISTON_PUSH = 'H';
    constexpr char PORT_LB_PISTON_PULL = 'G';

    constexpr int PORT_IMU = 9;
    constexpr double IMU_SCALE = 1.00524963699;

    constexpr int PORT_PARALLEL_TRACKER = -14;
    constexpr int PORT_PERPENDICULAR_TRACKER = 16;
    constexpr double PARALLEL_TRACKER_OFFSET = -104 + 3.08231636368;
    constexpr double PERPENDICULAR_TRACKER_OFFSET = -3.60498;
    constexpr double PARALLEL_TRACKER_DIAMETER = 2.75;
    constexpr double PERPENDICULAR_TRACKER_DIAMETER = 2.75;

    constexpr int PORT_DISTANCE_LEFT = 2;
    constexpr int PORT_DISTANCE_RIGHT = 6;
    constexpr int PORT_DISTANCE_FRONT = 4;

    constexpr double DRIVE_SLEW_RATE = (127.0 / 0.25) * (20.0/1000.0);
    constexpr double DRIVE_KP = 6.0;
    constexpr double DRIVE_KI = 0.0;
    constexpr double DRIVE_KI_RANGE = 1.0;
    // constexpr double DRIVE_KD = 36.5;
    constexpr double DRIVE_KD = 26;
    constexpr double TURN_KP = 4.0;
    // constexpr double TURN_KI = 1.5;
    constexpr double TURN_KI = 0.5;
    constexpr double TURN_KI_RANGE = 3.0;
    constexpr double TURN_KD = 21.5;

    std::shared_ptr<lib15442c::TankDrive> make_drivetrain();
    std::shared_ptr<mechanism::Arm> make_arm();
    std::shared_ptr<mechanism::RingManager> make_ring_manager(
        std::shared_ptr<mechanism::Arm> lb, std::shared_ptr<lib15442c::IPneumatic> lb_lift_push,
        std::shared_ptr<lib15442c::IPneumatic> lb_lift_pull, std::shared_ptr<lib15442c::IPneumatic> pto,
        std::shared_ptr<lib15442c::TankDrive> drivetrain
    );

    std::shared_ptr<lib15442c::TrackerOdom> make_tracker_odom();
    std::shared_ptr<lib15442c::DriveController> make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry);
}