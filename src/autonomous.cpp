#include "main.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

#define RUN_AUTO(auto_route) auto_route(drive_controller, drivetrain, odometry, ring_mech, clamp, oinker, alliance_stake_adjust, alliance)

// #define AUTO_SELECT gui::Route::NEGATIVE
// #define AUTO_SELECT_COLOR gui::AllianceColor::BLUE

void autonomous() {
	INFO_TEXT("Autonomous Start");

	double start_time = pros::millis() / 1000.0;

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

    std::shared_ptr<mechanism::RingMech> ring_mech = config::make_ring_mech();

    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic oinker = lib15442c::Pneumatic(config::PORT_OINKER);
    lib15442c::Pneumatic alliance_stake_adjust = lib15442c::Pneumatic(config::PORT_ALLIANCE_STAKE_ADJUST);

	drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::BRAKE);
    odometry->start_task();
	odometry->set_rotation(0_deg);
	
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
			RUN_AUTO(auto_routes::solo);
		} break;
		case gui::Route::SKILLS: {
			RUN_AUTO(auto_routes::skills);
		} break;
		default: break;
	}

	double end_time = pros::millis() / 1000.0;

	ring_mech->stop_task();
	odometry->stop_task();
	drivetrain->move(0, 0);
	
	INFO("Autonomous End. Took %.2fs", end_time - start_time);
}