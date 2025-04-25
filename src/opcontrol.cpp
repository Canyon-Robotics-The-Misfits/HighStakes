#include "main.h"
#include "config.h"

#include "mechanism/arm.hpp"
#include "mechanism/ring_manager.hpp"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"


#define LOGGER "opcontrol.cpp"

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
        bool intake_override = controller.get_digital(DIGITAL_Y);

        bool score = controller.get_digital(DIGITAL_R1);
        bool load = controller.get_digital(DIGITAL_R2);

        bool descore_1 = controller.get_digital(DIGITAL_X);
        bool descore_2 = controller.get_digital(DIGITAL_UP);

        if (intake || intake_reverse || score || load || descore_1 || descore_2)
        {
            rm->set_lb_override(false);
        }

        if (intake)
        {
            rm->intake();
        }
        else if (intake_override)
        {
            rm->intake_override();
        }
        else if (intake_reverse)
        {
            rm->intake_reverse();
        }
        else if (score)
        {
            rm->score();
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
            if (!(intake || intake_reverse || intake_override) && intake != intake_last)
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
        
        intake_last = intake || intake_reverse || intake_override;
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
	std::shared_ptr<lib15442c::Pneumatic> lb_lift_push = std::make_shared<lib15442c::Pneumatic>(config::PORT_LB_PISTON_PUSH, false, false);
	std::shared_ptr<lib15442c::Pneumatic> lb_lift_pull = std::make_shared<lib15442c::Pneumatic>(config::PORT_LB_PISTON_PULL, false, false);
    lib15442c::Pneumatic intake_lift = lib15442c::Pneumatic(config::PORT_INTAKE_LIFT);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    std::shared_ptr<mechanism::Arm> lb = config::make_arm();
    std::shared_ptr<mechanism::RingManager> rm = config::make_ring_manager(lb, lb_lift_push, lb_lift_pull);

    std::shared_ptr<lib15442c::TrackerOdom> tracker_odom = config::make_tracker_odom();
    // lib15442c::MCLOdom mcl_odom = lib15442c::MCLOdom(
    //     {
    //         particle_count: 2000,
    //         uniform_random_percent: 0.1,
    //         tracker_odom_sd: 0.05
    //     },
    //     tracker_odom,
    //     {
    //         { // front
    //             port: 1,
    //             x_offset: -5.5,
    //             y_offset: 7,
    //             theta_offset: 0,
    //         },
    //         { // back
    //             port: 1,
    //             x_offset: -5.5,
    //             y_offset: -5.5,
    //             theta_offset: M_PI,
    //         },
    //         { // left
    //             port: 1,
    //             x_offset: -5.25,
    //             y_offset: -3,
    //             theta_offset: M_PI / 2.0,
    //         },
    //         { // right
    //             port: 1,
    //             x_offset: 7,
    //             y_offset: -5,
    //             theta_offset: -M_PI / 2.0,
    //         }
    //     }
    // );

    clamp.retract();
    descore.retract();
    intake_lift.retract();

    rm->set_color_sort(mechanism::SortColor::BLUE);

    // tracker_odom->initialize(144 - 53 - 4, 13 + 1, 224_deg);
    tracker_odom->initialize(0, 0, 0_deg);


    // r1 score
    // r2 load
    // l1 toggle clamp
    // l2 intake

    // a reverse intake
    // b doinker
    // x descore 1
    // y intake override
    
    // up arrow descore 2
    // down arrow descore goal

    // int i = 0;
    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_ring_mech(controller, rm, lb);
        
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        // {
        //     intake_lift.toggle();
        // }
        
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        // {
        //     doinker.toggle();
        // }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {
            clamp.toggle();
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            descore.toggle();
        }
        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            lb_lift_push->toggle();
        }

        // i++;
        // if (i % 5 == 0) {
        //     std::cout << tracker_odom->get_x() << ", " << tracker_odom->get_y() << ", " << tracker_odom->get_rotation().deg_unwrapped() << std::endl;
        // }

        pros::delay(20);
    }
}