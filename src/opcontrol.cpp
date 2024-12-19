#include "main.h"
#include "config.h"

#include "mechanism/intake.h"

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

void control_intake(pros::Controller controller, std::shared_ptr<mechanism::Intake> intake)
{
    if (controller.get_digital(DIGITAL_L1))
    {
        intake->set_state(mechanism::IntakeState::HOOD);
    }
    else if (controller.get_digital(DIGITAL_L2))
    {
        intake->set_state(mechanism::IntakeState::REVERSE);
    }
    else
    {
        intake->set_state(mechanism::IntakeState::DISABLED);
    }
}

void control_clamp(pros::Controller controller, lib15442c::Pneumatic clamp)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        clamp.toggle();
    }
}

void control_oinker(pros::Controller controller, lib15442c::Pneumatic oinker)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        oinker.toggle();
    }
}

void opcontrol()
{
    INFO_TEXT("OPControl Start");

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    // std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic redirect = lib15442c::Pneumatic(config::PORT_REDIRECT);
    // lib15442c::Pneumatic oinker = lib15442c::Pneumatic(config::PORT_OINKER);
    // lib15442c::Pneumatic alliance_stake_adjust = lib15442c::Pneumatic(config::PORT_ALLIANCE_STAKE_ADJUST);

    // std::shared_ptr<lib15442c::TrackerOdom> tracker_odom = config::make_tracker_odom();
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

    // TEMP
    lib15442c::MotorGroup arm = lib15442c::MotorGroup(config::PARAMS_ARM, config::PORT_ARM);
    lib15442c::Motor intake = lib15442c::Motor(config::PARAMS_INTAKE);

    arm.set_brake_mode(lib15442c::MotorBrakeMode::HOLD);

    clamp.extend();

    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_clamp(controller, clamp);
        // control_oinker(controller, oinker);

        arm.move(controller.get_analog(ANALOG_RIGHT_Y));
        
        if (controller.get_digital(DIGITAL_R1))
        {
            intake.move(127);
        }
        else if (controller.get_digital(DIGITAL_R2))
        {
            intake.move(-127);
        }
        else
        {
            intake.move(0);
        }

        if (controller.get_digital_new_press(DIGITAL_L2))
        {
            redirect.toggle();
        }

        // std::cout << mcl_odom.get_x() << ", " << mcl_odom.get_y() << tracker_odom->get_x() << ", " << tracker_odom->get_y() << std::endl;

        pros::delay(20);
    }
}