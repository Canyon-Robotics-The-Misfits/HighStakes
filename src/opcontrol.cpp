#include "main.h"
#include "config.h"

#include "mechanism/ring_mech.h"

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

bool intake_on = true;
bool arm_on = false;
void control_ring_mech(pros::Controller controller, std::shared_ptr<mechanism::RingMech> ring_mech, lib15442c::Pneumatic alliance_stake_adjust)
{
    if (std::abs(controller.get_analog(ANALOG_RIGHT_Y)) < 10)
    {
        // intake toggle
        if (controller.get_digital_new_press(DIGITAL_Y))
        {
            intake_on = !intake_on;
            arm_on = false;
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
            alliance_stake_adjust.retract();
        }
        else if (controller.get_digital_new_press(DIGITAL_X))
        {
            intake_on = false;
            arm_on = true;
            ring_mech->set_state(mechanism::ARM_ALLIANCE_STAKE);
            alliance_stake_adjust.extend();
        }
        else if (controller.get_digital(DIGITAL_R2) && !arm_on) // reverse if r2 pressed
        {
            ring_mech->set_state(mechanism::INTAKE_OUTTAKE);
            alliance_stake_adjust.retract();
        }
        else if (controller.get_digital(DIGITAL_L2) && !arm_on) // control redirect with l2
        {
            ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
            alliance_stake_adjust.retract();
        }
        else if (intake_on && !arm_on)
        {
            ring_mech->set_state(mechanism::INTAKE_HOOD);
            alliance_stake_adjust.retract();
        }
        else if (!arm_on)
        {
            ring_mech->move_manual(0);
            ring_mech->set_state(mechanism::DISABLED);
            alliance_stake_adjust.retract();
        }

        // double raw_joystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // if (std::abs(raw_joystick) > 12)
        // {
        //     arm->move(raw_joystick);
        // }
        // else if (arm->get_target() == mechanism::ArmTarget::MANUAL)
        // {
        //     arm->move(0);
        // }
    } else {
        ring_mech->move_manual(controller.get_analog(ANALOG_RIGHT_Y));
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
    
    
	lib15442c::TrajectoryBuilder trajectory_builder = lib15442c::TrajectoryBuilder({ point: lib15442c::Vec(0.0, 0.0), tangent: lib15442c::Vec(0.0, 100.0) });
	trajectory_builder.append_hermite({ point: lib15442c::Vec(24.0, 48.0), tangent: lib15442c::Vec(0.0, 100.0) });
    // trajectory_builder.add_max_speed_zone(lib15442c::circle_zone(lib15442c::Vec(25, 25), 10, 60));
	auto trajectory = trajectory_builder.compute(config::TRAJECTORY_CONSTRAINTS, -1, true);
    // trajectory.debug_log();
    // std::cout << "-----" << std::endl;
    pros::delay(300);

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    std::shared_ptr<lib15442c::TankDrive> drivetrain = config::make_drivetrain();
    // std::shared_ptr<mechanism::RingMech> ring_mech = config::make_ring_mech();
    
    lib15442c::Pneumatic clamp = lib15442c::Pneumatic(config::PORT_CLAMP);
    // lib15442c::Pneumatic oinker = lib15442c::Pneumatic(config::PORT_OINKER);
    // lib15442c::Pneumatic alliance_stake_adjust = lib15442c::Pneumatic(config::PORT_ALLIANCE_STAKE_ADJUST);

    std::shared_ptr<lib15442c::TrackerOdom> tracker_odom = config::make_tracker_odom();

    lib15442c::RAMSETE ramsete = lib15442c::RAMSETE(trajectory, 0.0013, 0.7);

    ramsete.execute(drivetrain, tracker_odom);


    return;

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

    while (true)
    {
        control_drivetrain(controller, drivetrain);
        // control_ring_mech(controller, ring_mech, alliance_stake_adjust);
        control_clamp(controller, clamp);
        // control_oinker(controller, oinker);

        // std::cout << mcl_odom.get_x() << ", " << mcl_odom.get_y() << tracker_odom->get_x() << ", " << tracker_odom->get_y() << std::endl;

        pros::delay(20);
    }
}