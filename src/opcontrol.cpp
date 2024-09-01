#include "main.h"
#include "config.h"

#define LOGGER "opcontrol.cpp"

double curve_joystick(double in)
{
    constexpr double a = 70;
    constexpr double b = 147;

    if (in != 0)
    {
        float toT = fabs(in) / 127.0;
        float output = a * pow(1 - toT, 2) * toT + b * (1 - toT) * pow(toT, 2) + 127 * pow(toT, 3);
        return output * lib15442c::sgn(in);
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

    double linear_speed = curve_joystick(linear_raw);
    double rotational_speed = rotational_raw;

    drivetrain->move(linear_speed, rotational_speed);
}

void control_arm(pros::Controller controller, lib15442c::Motor arm)
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        arm.move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        arm.move(-127);
    }
    else
    {
        arm.move(0);
    }
}

void control_intake(pros::Controller controller, lib15442c::Motor intake)
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intake.move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.move(-127);
    }
    else
    {
        intake.move(0);
    }
}

void control_clamp(pros::Controller controller, lib15442c::Pneumatic clamp)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
        clamp.toggle();
    }
}

void control_redirect(pros::Controller controller, lib15442c::Pneumatic redirect, pros::Optical color_sensor)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        redirect.toggle();
    }

    double hue = color_sensor.get_hue();

    if (hue > 170 && hue < 230)
    {
        std::cout << "blue: " << hue << std::endl;

        redirect.retract();
    }
    else if (hue > 0 && hue < 20)
    {
        std::cout << "red: " << hue << std::endl;

        redirect.extend();
    }
}

void opcontrol()
{
	INFO_TEXT("OPControl Start");

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    lib15442c::Motor arm = config::make_arm();
    lib15442c::Motor intake = config::make_intake();
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic redirect = lib15442c::Pneumatic(config::PORT_REDIRECT);
    pros::Optical color_sensor = pros::Optical(config::PORT_OPTICAL);

    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_arm(controller, arm);
        control_intake(controller, intake);
        control_clamp(controller, clamp);
        control_redirect(controller, redirect, color_sensor);

        pros::delay(20);
    }
}