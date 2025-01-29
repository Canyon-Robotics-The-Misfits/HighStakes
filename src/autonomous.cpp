#include "main.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

// #define AUTO_SELECT gui::Route::NEGATIVE
// #define AUTO_SELECT_COLOR gui::AllianceColor::RED

void autonomous() {
	INFO_TEXT("Autonomous Start");

	double start_time = pros::millis() / 1000.0;

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    std::shared_ptr<mechanism::Arm> arm = config::make_arm(intake);
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic doinker = lib15442c::Pneumatic(config::PORT_DOINKER);

	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

	drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::BRAKE);
    // odometry->start_task();
	// odometry->set_rotation(0_deg);
	
	#ifndef AUTO_SELECT
	gui::ScreenGUI &gui = gui::ScreenGUI::access();
	#else
	#ifndef AUTO_SELECT_COLOR
	gui::ScreenGUI &gui = gui::ScreenGUI::access();
	#endif
	#endif

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
		case gui::Route::RIGHT_SAFE: {
			RUN_AUTO(auto_routes::right_safe);
		} break;
		case gui::Route::LEFT_SAFE: {
			RUN_AUTO(auto_routes::left_safe);
		} break;
		case gui::Route::POSITIVE: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO(auto_routes::positive_red);
			}
			else
			{
				RUN_AUTO(auto_routes::positive_blue);
			}
		} break;
		case gui::Route::NEGATIVE: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO(auto_routes::negative_red);
			}
			else
			{
				RUN_AUTO(auto_routes::negative_blue);
			}
		} break;
		case gui::Route::SOLO: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO(auto_routes::solo_red);
			}
			else
			{
				RUN_AUTO(auto_routes::solo_blue);
			}
		} break;
		case gui::Route::SKILLS: {
			RUN_AUTO(auto_routes::skills);
		} break;
		default: break;
	}

	double end_time = pros::millis() / 1000.0;

	intake->stop_task();
	arm->stop_task();
	odometry->stop_task();
	drivetrain->move(0, 0);
	
	INFO("Autonomous End. Took %.2fs", end_time - start_time);

	pros::delay(2000);
	INFO("Took %.2fs", end_time - start_time); // second log a second late to make sure any disconnects won't prevent it from logging
}