#include <memory>

#include "main.h"
#include "config.h"

#define LOGGER "config.cpp"

std::shared_ptr<lib15442c::TankDrive> config::make_drivetrain()
{
	std::shared_ptr<lib15442c::MotorGroup> left_drive = std::make_shared<lib15442c::MotorGroup>(
		config::PARAMS_LEFT_DRIVE,
		config::PORT_LEFT_DRIVE);
	std::shared_ptr<lib15442c::MotorGroup> right_drive = std::make_shared<lib15442c::MotorGroup>(
		config::PARAMS_RIGHT_DRIVE,
		config::PORT_RIGHT_DRIVE);

	std::shared_ptr<lib15442c::TankDrive> drivetrain = std::make_shared<lib15442c::TankDrive>(
		left_drive, right_drive,
		config::DRIVE_WHEEL_DIAMETER, 1.0, config::DRIVE_TRACK_WIDTH, config::FEEDFORWARD_CONSTANTS);

	if (!drivetrain->is_installed())
	{
		std::vector<int> uninstalled_parts = drivetrain->get_uninstalled_motors();

		ERROR_TEXT("Some drivetrain motors not detected!");

		for (int port : uninstalled_parts)
		{
			ERROR("port: %d", port);
		}
	}

	return drivetrain;
}

std::shared_ptr<mechanism::Intake> config::make_intake()
{
	auto motors = std::make_shared<lib15442c::Motor>(config::PARAMS_INTAKE);

    auto redirect = std::make_shared<lib15442c::Pneumatic>(config::PORT_REDIRECT, true, false);
    // auto color_sensor = std::make_shared<pros::Optical>(config::PORT_OPTICAL);

	return std::make_shared<mechanism::Intake>(motors, redirect);
}

std::shared_ptr<mechanism::Arm> config::make_arm(std::shared_ptr<mechanism::Intake> intake)
{
	auto motors = std::make_shared<lib15442c::MotorGroup>(config::PARAMS_ARM, config::PORT_ARM);

	auto rotation_sensor = std::make_shared<pros::Rotation>(config::PORT_ARM_ROTATION);
	auto pid = std::make_shared<lib15442c::PID>(config::PARAMS_ARM_PID);

	return std::make_shared<mechanism::Arm>(
		motors,
		rotation_sensor,
		intake,
		pid,
		config::ARM_TARGET_CONFIG,
		10.0
	);
}

std::shared_ptr<lib15442c::TrackerOdom> config::make_tracker_odom()
{
	// lib15442c::TrackerIMU imu_1 = {
	// 	imu : std::make_shared<pros::IMU>(config::PORT_IMU),
	// 	scale : config::IMU_SCALE
	// };
	lib15442c::TrackerIMU imu_2 = {
		imu : std::make_shared<pros::IMU>(config::PORT_IMU_2),
		scale : config::IMU_SCALE_2
	};

	lib15442c::TrackerWheel parallel = {
		tracker : std::make_shared<pros::Rotation>(config::PORT_PARALLEL_TRACKER),
		offset : config::PARALLEL_TRACKER_OFFSET,
		diameter : config::PARALLEL_TRACKER_DIAMETER
	};
	lib15442c::TrackerWheel perpendicular = {
		tracker : std::make_shared<pros::Rotation>(config::PORT_PERPENDICULAR_TRACKER),
		offset : config::PERPENDICULAR_TRACKER_OFFSET,
		diameter : config::PARALLEL_TRACKER_DIAMETER
	};

	return std::make_shared<lib15442c::TrackerOdom>(
		parallel,
		perpendicular,
		false,
		// imu_1,
		imu_2
	);
}

std::shared_ptr<lib15442c::DriveController> config::make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry)
{
	auto drive_pid = std::make_shared<lib15442c::PID>(lib15442c::PIDParameters {
		kP: config::DRIVE_KP,
		kI: config::DRIVE_KI,
		kD: config::DRIVE_KD,
		// slew_rate: config::DRIVE_SLEW_RATE
		integral_active_zone: config::DRIVE_KI_RANGE,
		integral_max: config::DRIVE_KI * 25,
		reset_integral_on_cross: true,
	});

	auto turn_pid = std::make_shared<lib15442c::PID>(lib15442c::PIDParameters {
		kP: config::TURN_KP,
		kI: config::TURN_KI,
		kD: config::TURN_KD,
		integral_active_zone: config::TURN_KI_RANGE,
		integral_max: config::TURN_KI * 15,
		reset_integral_on_cross: true,
	});

	return std::make_shared<lib15442c::DriveController>(
		drivetrain,
		odometry,
		drive_pid,
		turn_pid
	);
}