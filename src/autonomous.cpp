#include "main.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

#define AUTO_OVERRIDE RUN_AUTO(auto_routes::skills)

double start_time = -1;
void log_end_time()
{
	if (start_time != -1)
	{
		double end_time = pros::millis() / 1000.0;

		printf("Autonomous End. Took %.2fs\n", end_time - start_time);
		start_time = -1;
	}
}

void autonomous() {
	INFO_TEXT("Autonomous Start");

	start_time = pros::millis() / 1000.0;
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic descore = lib15442c::Pneumatic(config::PORT_DESCORE);
    lib15442c::Pneumatic doinker = lib15442c::Pneumatic(config::PORT_DOINKER);
    std::shared_ptr<lib15442c::Pneumatic> pto = std::make_shared<lib15442c::Pneumatic>(config::PORT_PTO);
	std::shared_ptr<lib15442c::Pneumatic> lb_lift_push = std::make_shared<lib15442c::Pneumatic>(config::PORT_LB_PISTON_PUSH, false, false);
	std::shared_ptr<lib15442c::Pneumatic> lb_lift_pull = std::make_shared<lib15442c::Pneumatic>(config::PORT_LB_PISTON_PULL, false, false);
    lib15442c::Pneumatic intake_lift = lib15442c::Pneumatic(config::PORT_INTAKE_LIFT);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    std::shared_ptr<mechanism::Arm> lb = config::make_arm();
    std::shared_ptr<mechanism::RingManager> rm = config::make_ring_manager(lb, lb_lift_push, lb_lift_pull, pto, drivetrain);

	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

	drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::BRAKE);
    // odometry->start_task();
	// odometry->set_rotation(0_deg);

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui::AllianceColor alliance = gui.get_alliance();

	switch (gui.get_selected_auto())
	{
		case gui::Route::NONE: {
			#ifdef AUTO_OVERRIDE
			AUTO_OVERRIDE;
			#endif
		} break;
		case gui::Route::POSITIVE: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO_PARAM(auto_routes::positive_red, false);
			}
			else
			{
				RUN_AUTO_PARAM(auto_routes::positive_blue, false);
			}
		} break;
		case gui::Route::NEGATIVE: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO_PARAM(auto_routes::negative_red, false);
			}
			else
			{
				RUN_AUTO_PARAM(auto_routes::negative_blue, false);
			}
		} break;
		case gui::Route::POSITIVE_ELIMS: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO_PARAM(auto_routes::positive_red, true);
			}
			else
			{
				RUN_AUTO_PARAM(auto_routes::positive_blue, true);
			}
		} break;
		case gui::Route::NEGATIVE_ELIMS: {
			if (alliance == gui::AllianceColor::RED)
			{
				RUN_AUTO_PARAM(auto_routes::negative_red, true);
			}
			else
			{
				RUN_AUTO_PARAM(auto_routes::negative_blue, true);
			}
		} break;
		case gui::Route::SKILLS: {
			RUN_AUTO(auto_routes::skills);
		} break;
		default: break;
	}

	rm->stop_task();
	lb->stop_task();
	odometry->stop_task();
	drivetrain->move(0, 0);
	
	log_end_time();

	pros::delay(2000);
	INFO("Took %.2fs", end_time - start_time); // second log a second late to make sure any disconnects won't prevent it from logging
}