#include "main.h"
#include "config.h"

#include "mechanism/intake.h"

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
    double rotational_raw = controller.get_analog(ANALOG_LEFT_X);

    if (sqrt(linear_raw * linear_raw + rotational_raw * rotational_raw) < 12)
    {
        linear_raw = 0;
        rotational_raw = 0;
    }

    double linear_speed = linear_raw;
    double rotational_speed = curve_joystick(rotational_raw);

    drivetrain->move(linear_speed, rotational_speed);
}

// a intake
// r2 redirect hold + intake forward, let go intake off + redirect off
// l2 intake reverse
bool intake_on = false;
void control_intake(pros::Controller controller, std::shared_ptr<mechanism::Intake> intake)
{
    if (controller.get_digital_new_press(DIGITAL_A))
    {
        intake_on = !intake_on;
    }

    if (controller.get_digital(DIGITAL_L2))
    {
        intake->set_state(mechanism::IntakeState::REVERSE);
    }
    else if (controller.get_digital(DIGITAL_R2))
    {
        intake->set_state(mechanism::IntakeState::WALL_STAKE);
    }
    else if (intake_on)
    {
        intake->set_state(mechanism::IntakeState::HOOD);
    }
    else 
    {
        intake->set_state(mechanism::IntakeState::DISABLED);
    }
}

// x alliance stake
// r1 arm
bool arm_manual = false;
void control_arm(pros::Controller controller, std::shared_ptr<mechanism::Arm> arm)
{
    if (std::abs(controller.get_analog(ANALOG_RIGHT_Y)) > 10)
    {
        arm->move_manual(controller.get_analog(ANALOG_RIGHT_Y));
        arm_manual = true;
    }
    else if (controller.get_digital_new_press(DIGITAL_R1))
    {
        if (arm->is_loading())
        {
            arm->set_state(mechanism::ArmState::NEUTRAL_STAKE);
        }
        else
        {
            arm->set_state(mechanism::ArmState::LOAD);
        }
    }
    else if (controller.get_digital_new_press(DIGITAL_X))
    {
        arm->set_state(mechanism::ArmState::ALLIANCE_STAKE);
    }
    else if (arm_manual)
    {
        arm->move_manual(0);
        arm_manual = false;
    }
}

// l1 clamp
void control_clamp(pros::Controller controller, lib15442c::Pneumatic clamp)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        clamp.toggle();
    }
}

void control_doinker(pros::Controller controller, lib15442c::Pneumatic doinker)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        doinker.toggle();
    }
}

void opcontrol()
{
    INFO_TEXT("OPControl Start");

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    std::shared_ptr<mechanism::Arm> arm = config::make_arm();
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic doinker = lib15442c::Pneumatic(config::PORT_DOINKER);

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

    clamp.extend();
    arm->set_state(mechanism::ArmState::DISABLED);

    // tracker_odom->initialize(57, 16, 137_deg);
    // tracker_odom->initialize(0, 0, 180_deg);
    // tracker_odom->set_perpendicular_offset(config::PERPENDICULAR_TRACKER_OFFSET_MOGO);

    int i = 0;

    // x alliance stake
    // b doinker
    // a intake

    // r1 arm
    // r2 redirect hold + intake forward, let go intake off + redirect off
    // l1 clamp
    // l2 intake reverse
    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_clamp(controller, clamp);
        control_intake(controller, intake);
        control_arm(controller, arm);
        control_doinker(controller, doinker);

        // i++;
        // if (i % 2 == 0) {
        //     std::cout << tracker_odom->get_x() << ", " << tracker_odom->get_y() << ", " << tracker_odom->get_rotation().deg() << std::endl;
        // }


        pros::delay(20);
    }
}