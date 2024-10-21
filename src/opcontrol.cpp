#include "main.h"
#include "config.h"

#include "mechanism/ring_mech.h"

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

bool intake_on = true;
bool arm_on = false;
void control_ring_mech(pros::Controller controller, std::shared_ptr<mechanism::RingMech> ring_mech)
{
    // intake toggle
    if (controller.get_digital_new_press(DIGITAL_Y))
    {
        intake_on = !intake_on;
    }

    if (controller.get_digital_new_press(DIGITAL_R2) || controller.get_digital_new_press(DIGITAL_L2))
    {
        arm_on = false;
    }

    if (controller.get_digital_new_press(DIGITAL_R1))
    {
        intake_on = false;
        arm_on = true;
        if (ring_mech->is_arm_loading())
        {
            ring_mech->set_state(mechanism::ARM_NEUTRAL_STAKE);
        }
        else
        {
            ring_mech->set_state(mechanism::ARM_LOAD);
        }
    }
    else if (controller.get_digital(DIGITAL_R2) && !arm_on) // reverse if r2 pressed
    {
        ring_mech->set_state(mechanism::INTAKE_OUTTAKE);
    }
    else if (controller.get_digital(DIGITAL_L2) && !arm_on) // control redirect with l2
    {
        ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    }
    else if (intake_on && !arm_on)
    {
        ring_mech->set_state(mechanism::INTAKE_HOOD);
    }
    else if (!arm_on)
    {
        ring_mech->set_state(mechanism::DISABLED);
    }

    // double raw_joystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // if (abs(raw_joystick) > 12)
    // {
    //     arm->move(raw_joystick);
    // }
    // else if (arm->get_target() == mechanism::ArmTarget::MANUAL)
    // {
    //     arm->move(0);
    // }
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
    std::shared_ptr<mechanism::RingMech> ring_mech = config::make_ring_mech();
    
    // std::shared_ptr<lib15442c::TrackerOdom> odometry = config::make_tracker_odom();
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    lib15442c::Pneumatic oinker = lib15442c::Pneumatic(config::PORT_OINKER);
    lib15442c::Pneumatic alliance_stake_adjust = lib15442c::Pneumatic(config::PORT_ALLIANCE_STAKE_ADJUST);

    // odometry->startTask();
    clamp.extend();

    // int tick = 0;
    while (true)
    {
        control_drivetrain(controller, drivetrain);
        control_ring_mech(controller, ring_mech);
        control_clamp(controller, clamp);
        control_oinker(controller, oinker);

        if (controller.get_digital_new_press(DIGITAL_A))
        {
            alliance_stake_adjust.toggle();
        }

        // if (tick % 5 == 0)
        // {
        //     std::cout << odometry->getX() << ", " << odometry->getY() << std::endl;
        // }

        // tick+=1;
        pros::delay(20);
    }
}