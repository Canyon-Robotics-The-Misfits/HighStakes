#include "main.h"
#include "config.h"
#include "autonomous.h"

#include "mechanism/arm.hpp"
#include "mechanism/ring_manager.hpp"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"


#define LOGGER "opcontrol.cpp"

void descore_macro_test(lib15442c::Pneumatic descore, std::shared_ptr<lib15442c::Pneumatic> lb_lift_push, std::shared_ptr<mechanism::RingManager> rm, std::shared_ptr<mechanism::Arm> lb)
{
    descore.extend();
    rm->idle();
    pros::delay(500);

    rm->intake();
    pros::delay(100);
    rm->intake_reverse();

    pros::delay(300);

    // turn->await();

    rm->intake_hold();
    lb_lift_push->extend();
    rm->set_lb_override(true);
    lb->move(-60);
    pros::delay(350);
    lb->move(127);
    pros::delay(300);
    lb->move(0);
    rm->set_lb_override(false);
    lb_lift_push->retract();

    descore.retract();
    rm->idle();
}

double curve_joystick(double in)
{
    constexpr double a = 0.423851;
    constexpr double b = 0.0;
    constexpr double c = -1.71879;
    constexpr double d = 2.06431;
    constexpr double e = 0.230621;

    if (in != 0)
    {
        double t = std::abs(in) / 127.0;

        double out_normalized = a * pow(t, 5) + b * pow(t, 4) + c * pow(t, 3) + d * pow(t, 2) + e * t;

        return out_normalized * 127.0 * lib15442c::sgn(in);
    }
    else
    {
        return 0;
    }
}

void control_drivetrain(pros::Controller controller, std::shared_ptr<lib15442c::TankDrive> drivetrain)
{
    double linear_raw = controller.get_analog(ANALOG_LEFT_Y);
    // double linear_raw = 0;
    double rotational_raw = controller.get_analog(ANALOG_LEFT_X);

    if (linear_raw * linear_raw + rotational_raw * rotational_raw < 12 * 12)
    {
        linear_raw = 0;
        rotational_raw = 0;
    }

    double linear_speed = linear_raw;
    double rotational_speed = curve_joystick(rotational_raw);

    drivetrain->move(linear_speed, rotational_speed);
}

// r1 score
// r2 load
// l2 intake

// a reverse intake
// x descore
// y descore 2
bool joystick_override_last = false;
bool intake_last = false;
bool load_last = false;
bool score_last = false;
void control_ring_mech(pros::Controller controller, std::shared_ptr<mechanism::RingManager> rm, std::shared_ptr<mechanism::Arm> lb)
{
    bool joystick_override = abs(controller.get_analog(ANALOG_RIGHT_Y)) > 10;

    if (joystick_override)
    {
        rm->set_lb_override(true);
        lb->move(controller.get_analog(ANALOG_RIGHT_Y));
    }
    else if (joystick_override != joystick_override_last)
    {
        lb->move(0);
    }

    joystick_override_last = joystick_override;

    if (!joystick_override)
    {
        bool intake = controller.get_digital(DIGITAL_L2);
        bool intake_reverse = controller.get_digital(DIGITAL_A);
        bool intake_high_stake = controller.get_digital(DIGITAL_B);

        bool any_intake = intake || intake_reverse || intake_high_stake;

        bool score = controller.get_digital(DIGITAL_R1);
        bool load = controller.get_digital(DIGITAL_R2);

        // bool descore_1 = controller.get_digital(DIGITAL_B);
        bool descore_1 = false;
        // bool descore_2 = controller.get_digital(DIGITAL_UP);
        bool descore_2 = false;

        if (intake || intake_reverse || intake_high_stake || score || load || descore_1 || descore_2)
        {
            rm->set_lb_override(false);
        }

        if (intake)
        {
            rm->intake();
        }
        else if (intake_reverse)
        {
            rm->intake_reverse();
        }
        else if (intake_high_stake)
        {
            if (any_intake != intake_last)
            {
                rm->intake_high_stake();
            }
        }
        else if (score)
        {
            rm->score_skills();
        }
        else if (load)
        {
            rm->load();
        }
        else if (descore_1)
        {
            rm->descore_1();
        }
        else if (descore_2)
        {
            rm->descore_2();
        }
        else
        {
            if (!any_intake && any_intake != intake_last)
            {
                rm->stop_intake();
            }
            if (!load && load != load_last)
            {
                rm->stop_load();
            }
            if (!score && score != score_last)
            {
                rm->idle();
            }
        }
        
        intake_last = any_intake;
        load_last = load;
        score_last = score;
    }
}

void opcontrol()
{
    INFO_TEXT("OPControl Start");

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic descore = lib15442c::Pneumatic(config::PORT_DESCORE);
    lib15442c::Pneumatic doinker = lib15442c::Pneumatic(config::PORT_DOINKER);
    std::shared_ptr<lib15442c::Pneumatic> pto = std::make_shared<lib15442c::Pneumatic>(config::PORT_PTO);
	std::shared_ptr<lib15442c::Pneumatic> lb_lift_push = std::make_shared<lib15442c::Pneumatic>(config::PORT_LB_PISTON_PUSH, false, true);
	std::shared_ptr<lib15442c::Pneumatic> lb_lift_pull = std::make_shared<lib15442c::Pneumatic>(config::PORT_LB_PISTON_PULL, false, true);
    lib15442c::Pneumatic intake_lift = lib15442c::Pneumatic(config::PORT_INTAKE_LIFT);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    std::shared_ptr<mechanism::Arm> lb = config::make_arm();
    std::shared_ptr<mechanism::RingManager> rm = config::make_ring_manager(lb, lb_lift_push, lb_lift_pull, pto, drivetrain);

	std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
	std::shared_ptr<lib15442c::DriveController> drive_controller = config::make_drive_controller(drivetrain, odometry);

	gui::ScreenGUI &gui = gui::ScreenGUI::access();
    gui::AllianceColor alliance = gui.get_alliance();
    if (gui.get_selected_auto() == gui::Route::SKILLS)
    {
        drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::BRAKE);

        RUN_AUTO(auto_routes::skills);
    }

    clamp.retract();
    descore.retract();
    intake_lift.retract();
    doinker.retract();

    rm->set_color_sort(mechanism::SortColor::NONE);

    // tracker_odom->initialize(144 - 53 - 4, 13 + 1, 224_deg);
    // odometry->initialize(0, 0, 0_deg);


    // r1 score
    // r2 load
    // l1 toggle clamp
    // l2 intake

    // a reverse intake
    // b doinker
    // x descore 1
    // y intake override
    
    // up arrow climb
    // down arrow descore goal
    // left arrow lb lift
    // right arrow pto

    // int i = 0;
    bool current_lb_lift_state = false;
    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_ring_mech(controller, rm, lb);
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            if (rm->get_state() != mechanism::RingManagerState::PREP_CLIMB)
            {
                rm->prep_climb();
            }
            else
            {
                rm->idle();
            }
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            descore_macro_test(descore, lb_lift_push, rm, lb);
            // doinker.toggle();
            // intake_lift.toggle();
            
            // drive_controller->drive(1, { min_speed: 30, chained: true});
            // auto descore_turn_one = drive_controller->face_angle(180_deg, { min_speed: 40, chained: true, async: true });
            // descore_macro_test(descore, lb_lift_push, rm, lb);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {
            clamp.toggle();
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            rm->climb();
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            current_lb_lift_state = !current_lb_lift_state;

            if (current_lb_lift_state == true)
            {
                lb_lift_push->extend();
                lb_lift_pull->retract();
            }
            else
            {
                lb_lift_push->retract();
                lb_lift_pull->extend();
            }
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            pto->toggle();
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
            rm->climb(1);
        }

        // i++;
        // if (i % 5 == 0) {
        //     std::cout << lb->get_current_angle().deg() << std::endl;
        // }

        pros::delay(20);
    }
}