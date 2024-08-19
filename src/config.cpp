#include <memory>

#include "main.h"
#include "config.h"

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

	return drivetrain;
}