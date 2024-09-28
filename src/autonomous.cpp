#include "main.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

#define AUTO_SELECT gui::Route::POSITIVE
#define AUTO_SELECT_COLOR gui::AllianceColor::BLUE

void autonomous() {
	INFO_TEXT("Autonomous Start");

	double start_time = pros::millis() / 1000.0;

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

    std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    std::shared_ptr<mechanism::Arm> arm = config::make_arm();

    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic oinker = lib15442c::Pneumatic(config::PORT_OINKER);

	drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::BRAKE);
    odometry->startTask();
	odometry->setRotation(0_deg);
	intake->set_redirect_mode(mechanism::IntakeRedirectMode::NONE);
	arm->set_target(mechanism::ArmTarget::MANUAL);
	arm->move(0);
	
	gui::ScreenGUI &gui = gui::ScreenGUI::access();
	#ifndef AUTO_SELECT_COLOR
	gui::AllianceColor alliance = gui.get_alliance();
	#else
	gui::AllianceColor alliance = AUTO_SELECT_COLOR;
	#endif

	#ifndef AUTO_SELECT
	switch (gui.get_selected_auto())
	#else
	switch (AUTO_SELECT)
	#endif
	{
		case gui::Route::POSITIVE: {
			if (alliance == gui::AllianceColor::RED)
			{
				auto_routes::positive_red(drive_controller, drivetrain, odometry, intake, arm, clamp, oinker, alliance);
			}
			else
			{
				auto_routes::positive_blue(drive_controller, drivetrain, odometry, intake, arm, clamp, oinker, alliance);
			}
		} break;
		case gui::Route::NEGATIVE: {
			if (alliance == gui::AllianceColor::RED)
			{
				auto_routes::negative_red(drive_controller, drivetrain, odometry, intake, arm, clamp, oinker, alliance);
			}
			else
			{
				auto_routes::negative_blue(drive_controller, drivetrain, odometry, intake, arm, clamp, oinker, alliance);
			}
		} break;
		case gui::Route::SOLO: {
			auto_routes::solo(drive_controller, drivetrain, odometry, intake, arm, clamp, oinker, alliance);
		} break;
		case gui::Route::SKILLS: {
			auto_routes::skills(drive_controller, drivetrain, odometry, intake, arm, clamp, oinker, alliance);
		} break;
		default: break;
	}

	double end_time = pros::millis() / 1000.0;

	intake->stop_task();
	arm->stop_task();
	odometry->stopTask();
	drivetrain->move(0, 0);
	
	INFO("Autonomous End. Took %.2fs", end_time - start_time);
}