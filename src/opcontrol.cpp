#include "main.h"
#include "config.h"

#define LOGGER "opcontrol.cpp"

double curve_joystick(double in)
{
    constexpr double a = 0.346767;
    constexpr double b = 0.137;
    constexpr double c = 0;
    constexpr double d = 0.102864;
    constexpr double e = 0.413369;

    if (in != 0)
    {
        double t = in / 127.0;

        double out_normalized = a * pow(t, 5) + b * pow(t, 4) + c * pow(t, 3) + d * pow(t, 2) + e * t;
        
        return out_normalized * 127.0;
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

void control_arm(pros::Controller controller, lib15442c::Motor arm)
{
    double raw_joystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    if (raw_joystick > 12)
    {
        arm.move(raw_joystick);
    }
    else{
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
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        clamp.toggle();
    }
}

void control_redirect(pros::Controller controller, lib15442c::Pneumatic redirect, pros::Optical color_sensor)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
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