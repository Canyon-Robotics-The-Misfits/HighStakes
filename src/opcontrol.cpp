#include "main.h"
#include "config.h"

#define LOGGER "opcontrol.cpp"

double curve_joystick(double in)
{
    constexpr double a = 0.423851;
    constexpr double b = 0;
    constexpr double c = -1.71879;
    constexpr double d = 2.06431;
    constexpr double e = 0.230621;

    if (in != 0)
    {
        double t = fabs(in) / 127.0;

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

void control_arm(pros::Controller controller, std::shared_ptr<mechanism::Arm> arm)
{
    double raw_joystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    if (fabs(raw_joystick) > 12)
    {
        arm->move(raw_joystick);
    }
    else{
        arm->move(0);
    }
}

void control_intake(pros::Controller controller, std::shared_ptr<mechanism::Intake> intake)
{
    // Intake and Outtake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intake->move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake->move(-127);
    }
    else
    {
        intake->move(0);
    }

    // Redirect
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
        intake->set_redirect_mode(mechanism::IntakeRedirectMode::ALL);
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        intake->set_redirect_mode(mechanism::IntakeRedirectMode::NONE);
    }
    else
    {
        intake->set_redirect_mode(mechanism::IntakeRedirectMode::RED);
    }
}

void control_clamp(pros::Controller controller, lib15442c::Pneumatic clamp)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        clamp.toggle();
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

    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_arm(controller, arm);
        control_intake(controller, intake);
        control_clamp(controller, clamp);

        pros::delay(20);
    }
}