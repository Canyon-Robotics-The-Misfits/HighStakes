#include "ring_mech.h"
#include "lib15442c/logger.hpp"

#define LOGGER "ring_mech.cpp"

mechanism::RingMech::RingMech(RingMechParams params)
    : motors(params.motors),
      intake_redirect(params.intake_redirect),
      intake_optical(params.intake_optical),
      arm_rotation_sensor(params.arm_rotation_sensor),
      arm_limit(params.arm_limit),
      arm_pid(params.arm_pid),
      arm_target_config(params.arm_target_config)
{
    if (!motors->is_installed())
    {
        ERROR_TEXT("Arm/Intake motor(s) unplugged!");
    }
    if (!intake_optical->is_installed())
    {
        ERROR("Intake optical sensor is not detected on port %d!", intake_optical->get_port());
    }
    if (!arm_rotation_sensor->is_installed())
    {
        ERROR("Arm rotation sensor is not detected on port %d!", arm_rotation_sensor->get_port());
    }

    start_task();
}

bool mechanism::RingMech::is_intake()
{
    return (state & 0x10000) > 0 && (state & 0x01000) > 0;
}

bool mechanism::RingMech::is_arm()
{
    return (state & 0x10000) > 0 && (state & 0x01000) == 0;
}

void mechanism::RingMech::start_task()
{

    mutex.lock();
    if (task_on_flag)
    {
        mutex.unlock();
        return;
    }

    task_on_flag = true;
    mutex.unlock();

    task = pros::Task([this]()
                      {
        
        while (true)
        {
            mutex.lock();
            if (task_on_flag == false || ((pros::c::competition_get_status() & COMPETITION_DISABLED) != 0))
            {
                break;
            }
            mutex.unlock();
            
            if (is_arm())
            {
                motors->set_brake_mode(lib15442c::MotorBrakeMode::HOLD);

                double current_angle = 360.0 - (arm_rotation_sensor->get_angle() / 100.0); // divide by 100 to convert centidegrees to degrees

                // make 359 equal to 0
                if (current_angle > 180)
                {
                    current_angle = 0;
                }

                double target_angle = INFINITY;
                
                // std::cout << state << std::endl;
                
                switch (state)
                {
                case RingMechState::ARM_LOAD:
                {
                    target_angle = arm_target_config.load;
                }
                break;
                case RingMechState::ARM_ALLIANCE_STAKE:
                {
                    target_angle = arm_target_config.alliance_stake;
                }
                break;
                case RingMechState::ARM_NEUTRAL_STAKE:
                {
                    target_angle = arm_target_config.neutral_stake;
                }
                break;
                case RingMechState::ARM_LADDER_TOUCH:
                {
                    target_angle = arm_target_config.ladder_touch;
                }
                break;
                default: break;
                }

                
                if (target_angle != INFINITY)
                {
                    double output = -arm_pid->calculate(current_angle, target_angle);
                    // std::cout << current_angle << ", " << target_angle << ", " << output << std::endl;

                    // if (arm_limit->arm_limit->get_value() == true)
                    // {
                    //     output = std::max(output, 0.0);
                    // }

                    motors->move(output);
                }
            }
            else if (is_intake())
            {
                motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

                if (state == INTAKE_SORT_BLUE || state == INTAKE_SORT_RED)
                {
                    intake_optical->set_led_pwm(100);
                    motors->move(127.0 / 2.0); 
                }
                else
                {
                    intake_optical->set_led_pwm(0);

                    if (state == INTAKE_OUTTAKE)
                    {
                        motors->move(-127); 
                    }
                    else
                    {
                        motors->move(127); 
                    }
                }


                double hue = intake_optical->get_hue();

                switch (state)
                {
                case INTAKE_HOOD:
                {
                    intake_redirect->extend();
                    break;
                }
                case INTAKE_WALL_STAKE:
                {
                    intake_redirect->retract();
                    break;
                }
                case INTAKE_OUTTAKE:
                {
                    intake_redirect->extend();
                    break;
                }
                case INTAKE_SORT_RED:
                {
                    if (hue > 170 && hue < 250) // Check if the ring is blue
                    {
                        intake_redirect->retract();
                    }
                    else
                    {
                        intake_redirect->extend();
                    }
                    break;
                }
                case INTAKE_SORT_BLUE:
                {
                    if ((hue > 0 && hue < 30) || (hue > 350 && hue < 360)) // Check if the ring is red
                    {
                        intake_redirect->retract();
                    }
                    else
                    {
                        intake_redirect->extend();
                    }
                    break;
                }
                default: break;
                }
            }
            else
            {
                motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
                motors->move(0);
            }

            pros::delay(20);
        } });
}

void mechanism::RingMech::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::RingMech::set_state(mechanism::RingMechState state)
{
    mutex.lock();
    this->state = state;
    mutex.unlock();
}

mechanism::RingMechState mechanism::RingMech::get_state()
{
    mutex.lock();
    auto temp = state;
    mutex.unlock();

    return temp;
}

bool mechanism::RingMech::is_arm_loading()
{
    double current = 360.0 - (arm_rotation_sensor->get_angle() / 100.0);

    if (current > 180.0)
    {
        current = 0.0;
    }

    return abs(current - arm_target_config.load) < 10.0;
}

void mechanism::RingMech::move_manual(double voltage)
{
    set_state(mechanism::DISABLED);

    mutex.lock();
    motors->move(voltage);
    mutex.unlock();
}