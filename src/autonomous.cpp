#include "main.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

#define AUTO_OVERRIDE auto_routes::mp_test

void autonomous() {
	INFO_TEXT("Autonomous Start");

	double start_time = pros::millis() / 1000.0;

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    // std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    // std::shared_ptr<mechanism::Arm> arm = config::make_arm(intake);
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic doinker = lib15442c::Pneumatic(config::PORT_DOINKER);
    lib15442c::Pneumatic intake_lift = lib15442c::Pneumatic(config::PORT_INTAKE_LIFT);

	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

	drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::BRAKE);
    // odometry->start_task();
	// odometry->set_rotation(0_deg);

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui::AllianceColor alliance = gui.get_alliance();

	// switch (gui.get_selected_auto())
	// {
	// 	case gui::Route::NONE: {
	// 		#ifdef AUTO_OVERRIDE
	// 		RUN_AUTO(AUTO_OVERRIDE);
	// 		#endif
	// 	} break;
	// 	case gui::Route::POSITIVE: {
	// 		if (alliance == gui::AllianceColor::RED)
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::positive_red, false);
	// 		}
	// 		else
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::positive_blue, false);
	// 		}
	// 	} break;
	// 	case gui::Route::NEGATIVE: {
	// 		if (alliance == gui::AllianceColor::RED)
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::negative_red, false);
	// 		}
	// 		else
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::negative_blue, false);
	// 		}
	// 	} break;
	// 	case gui::Route::POSITIVE_ELIMS: {
	// 		if (alliance == gui::AllianceColor::RED)
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::positive_red, true);
	// 		}
	// 		else
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::positive_blue, true);
	// 		}
	// 	} break;
	// 	case gui::Route::NEGATIVE_ELIMS: {
	// 		if (alliance == gui::AllianceColor::RED)
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::negative_red, true);
	// 		}
	// 		else
	// 		{
	// 			RUN_AUTO_PARAM(auto_routes::negative_blue, true);
	// 		}
	// 	} break;
	// 	case gui::Route::SKILLS: {
	// 		RUN_AUTO(auto_routes::skills);
	// 	} break;
	// 	default: break;
	// }

	double end_time = pros::millis() / 1000.0;

	// intake->stop_task();
	// arm->stop_task();
	odometry->stop_task();
	drivetrain->move(0, 0);
	
	// INFO("Autonomous End. Took %.2fs", end_time - start_time);
	printf("Autonomous End. Took %.2fs", end_time - start_time);

	pros::delay(2000);
	INFO("Took %.2fs", end_time - start_time); // second log a second late to make sure any disconnects won't prevent it from logging
}