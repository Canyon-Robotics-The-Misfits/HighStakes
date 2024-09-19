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
		config::DRIVE_WHEEL_DIAMETER, 1, config::DRIVE_TRACK_WIDTH);

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

std::shared_ptr<mechanism::Arm> config::make_arm()
{
	auto motor = std::make_shared<lib15442c::Motor>(config::PARAMS_ARM);
	auto rotation_sensor = std::make_shared<pros::Rotation>(config::PORT_ARM_ROTATION);
	auto limit_switch = std::make_shared<pros::adi::DigitalIn>(config::PORT_ARM_LIMIT);
	auto pid = std::make_shared<lib15442c::PID>(config::PARAMS_ARM_PID);

	return std::make_shared<mechanism::Arm>(
		motor, rotation_sensor, limit_switch, pid, config::ARM_TARGET_CONFIG
	);
}

std::shared_ptr<mechanism::Intake> config::make_intake()
{
	auto intake = std::make_shared<lib15442c::Motor>(config::PARAMS_INTAKE);
    auto redirect = std::make_shared<lib15442c::Pneumatic>(config::PORT_REDIRECT);
    auto color_sensor = std::make_shared<pros::Optical>(config::PORT_OPTICAL);

	return std::make_shared<mechanism::Intake>(
		intake, redirect, color_sensor
	);
}

std::shared_ptr<lib15442c::TrackerOdom> config::make_tracker_odom()
{
	lib15442c::TrackerIMU imu = {
		imu : std::make_shared<pros::IMU>(config::PORT_IMU),
		scale : config::IMU_SCALE
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
		imu
	);
}

std::shared_ptr<lib15442c::DriveController> config::make_drive_controller(std::shared_ptr<lib15442c::IDrivetrain> drivetrain, std::shared_ptr<lib15442c::IOdometry> odometry)
{
	auto drive_pid = std::make_shared<lib15442c::PID>(lib15442c::PIDParameters {
		kP: config::DRIVE_KP,
		kI: config::DRIVE_KI,
		kD: config::DRIVE_KD,
		slew_rate: config::DRIVE_SLEW_RATE
	});

	auto turn_pid = std::make_shared<lib15442c::PID>(lib15442c::PIDParameters {
		kP: config::TURN_KP,
		kI: config::TURN_KI,
		kD: config::TURN_KD,
	});

	return std::make_shared<lib15442c::DriveController>(
		drivetrain,
		odometry,
		drive_pid,
		turn_pid
	);
}