#include <memory>

#include "main.h"
#include "config.h"

#define LOGGER "config.cpp"

std::shared_ptr<lib15442c::TankDrive> config::make_drivetrain()
{
	std::shared_ptr<lib15442c::MotorGroup> left_drive = std::make_shared<lib15442c::MotorGroup>(
		config::PARAMS_LEFT_DRIVE,
		config::PORT_LEFT_DRIVE
	);
	std::shared_ptr<lib15442c::MotorGroup> right_drive = std::make_shared<lib15442c::MotorGroup>(
		config::PARAMS_RIGHT_DRIVE,
		config::PORT_RIGHT_DRIVE
	);

	std::shared_ptr<lib15442c::TankDrive> drivetrain = std::make_shared<lib15442c::TankDrive>(
		left_drive, right_drive,
		config::DRIVE_WHEEL_DIAMETER, 1, config::DRIVE_TRACK_WIDTH
	);

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

lib15442c::Motor config::make_arm()
{
	auto arm = lib15442c::Motor(config::PARAMS_ARM);

	if (!arm.is_installed())
	{
		ERROR("Arm motor is not detected on port %d!", arm.get_port());
	}

	return arm;
}

lib15442c::Motor config::make_intake()
{
	auto intake = lib15442c::Motor(config::PARAMS_INTAKE);

	if (!intake.is_installed())
	{
		ERROR("Intake motor is not detected on port %d!", intake.get_port());
	}

	return intake;
}