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
        double t = abs(in) / 127.0;

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
    if (controller.get_digital_new_press(DIGITAL_R1))
    {
        if (arm->get_target() == mechanism::ArmTarget::LOAD)
        {
            arm->set_target(mechanism::ArmTarget::NEUTRAL_STAKE);
        }
        else
        {
            arm->set_target(mechanism::ArmTarget::LOAD);
        }
    }

    double raw_joystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    if (abs(raw_joystick) > 12)
    {
        arm->move(raw_joystick);
    }
    else if (arm->get_target() == mechanism::ArmTarget::MANUAL)
    {
        arm->move(0);
    }
}

bool intake_on = true;
bool color_sort = false;
void control_intake(pros::Controller controller, std::shared_ptr<mechanism::Intake> intake, std::shared_ptr<mechanism::Arm> arm)
{
    // intake toggle
    if (controller.get_digital_new_press(DIGITAL_Y))
    {
        intake_on = !intake_on;
    }

    // reverse if r2 pressed
    if (controller.get_digital(DIGITAL_R2))
    {
        intake->move(-127);
    }
    else if (intake_on)
    {
        intake->move(127);
    }
    else
    {
        intake->move(0);
    }
    
    // toggle color sort with x
    bool x_new_press = controller.get_digital_new_press(DIGITAL_X);
    if (x_new_press)
    {
        color_sort = !color_sort;
    
        if (color_sort)
        {
            arm->set_target(mechanism::ArmTarget::COLOR_SORT);
            controller.rumble(".");
        }
        else
        {
            controller.rumble("..");
        }
    }

    // control redirect with l2 and color sort
    if (controller.get_digital(DIGITAL_L2))
    {
        intake->set_redirect_mode(mechanism::IntakeRedirectMode::ALL); 
    }
    else if (color_sort)
    {
        if (x_new_press)
        {
            intake->set_redirect_mode(mechanism::IntakeRedirectMode::NONE);
        }
        else 
        {
            intake->set_redirect_mode(mechanism::IntakeRedirectMode::BLUE);
        }
    }
    else
    {
        intake->set_redirect_mode(mechanism::IntakeRedirectMode::NONE);
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
    std::shared_ptr<mechanism::Intake> intake = config::make_intake();
    std::shared_ptr<mechanism::Arm> arm = config::make_arm();
    
    std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic oinker = lib15442c::Pneumatic(config::PORT_OINKER);

    odometry->startTask();
    arm->set_target(mechanism::ArmTarget::MANUAL);

    // int tick = 0;
    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_arm(controller, arm);
        control_intake(controller, intake, arm);
        control_clamp(controller, clamp);
        control_oinker(controller, oinker);

        // if (tick % 5 == 0)
        // {
        //     std::cout << odometry->getX() << ", " << odometry->getY() << std::endl;
        // }

        // tick+=1;
        pros::delay(20);
    }
}