#include "main.h"
#include "gui/gui.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

#define AUTO_SELECT auto_routes::Route::SKILLS

void autonomous() {
	INFO_TEXT("Autonomous Start");

	double start_time = pros::millis() / 1000.0;

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

    std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    std::shared_ptr<mechanism::Arm> arm = config::make_arm();
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);

	arm->set_target(mechanism::ArmTarget::LOAD);
	odometry->setRotation(0_deg);

	#ifndef AUTO_SELECT
	switch (gui.get_selected_auto())
	#else
	switch (AUTO_SELECT)
	#endif
	{
		case auto_routes::Route::POSITIVE: {
			auto_routes::positive(drive_controller, drivetrain, odometry, intake, arm, clamp);
		} break;
		case auto_routes::Route::NEGATIVE: {
			auto_routes::negative(drive_controller, drivetrain, odometry, intake, arm, clamp);
		} break;
		case auto_routes::Route::POSITIVE_ELIMS: {
			auto_routes::positive_elims(drive_controller, drivetrain, odometry, intake, arm, clamp);
		} break;
		case auto_routes::Route::NEGATIVE_ELIMS: {
			auto_routes::negative_elims(drive_controller, drivetrain, odometry, intake, arm, clamp);
		} break;
		case auto_routes::Route::SOLO: {
			auto_routes::solo(drive_controller, drivetrain, odometry, intake, arm, clamp);
		} break;
		case auto_routes::Route::SKILLS: {
			auto_routes::skills(drive_controller, drivetrain, odometry, intake, arm, clamp);
		} break;
		default: break;
	}

	double end_time = pros::millis() / 1000.0;

	intake->stop_task();
	arm->stop_task();
	
	INFO("Autonomous End. Took %.2fs", end_time - start_time);
}