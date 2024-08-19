#include "main.h"
#include "config.h"

double curve_joystick(double in) {
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

void opcontrol()
{
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();

    while (true)
    {
        control_drivetrain(controller, drivetrain);

        pros::delay(20);
    }
}