#include "arm.h"
#include "lib15442c/logger.hpp"
#include "pros/misc.h"

#define LOGGER "arm.cpp"

mechanism::Arm::Arm(std::shared_ptr<lib15442c::IMotor> motors, std::shared_ptr<pros::Rotation> arm_rotation_sensor, std::shared_ptr<lib15442c::PID> arm_pid, ArmTargetConfig target_config)
    : motors(motors),
      arm_rotation_sensor(arm_rotation_sensor),
      arm_pid(arm_pid),
      target_config(target_config)
{
    if (!motors->is_installed())
    {
        ERROR_TEXT("Arm motor(s) unplugged!");
    }
    if (!arm_rotation_sensor->is_installed())
    {
        ERROR("Arm rotation sensor is not detected on port %d!", arm_rotation_sensor->get_port());
    }

    start_task();
}

void mechanism::Arm::start_task()
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
            
            motors->set_brake_mode(lib15442c::MotorBrakeMode::HOLD);

            double current_angle = 360.0 - (arm_rotation_sensor->get_angle() / 100.0); // divide by 100 to convert centidegrees to degrees

            // make 359 equal to 0
            if (current_angle > 180)
            {
                current_angle = 0;
            }

            double target_angle = INFINITY;
                
            switch (state)
            {
            case ArmState::LOAD:
            {
                target_angle = target_config.load;
            }
            break;
            case ArmState::ALLIANCE_STAKE:
            {
                target_angle = target_config.alliance_stake;
            }
            break;
            case ArmState::NEUTRAL_STAKE:
            {
                target_angle = target_config.neutral_stake;
            }
            break;
            case ArmState::LADDER_TOUCH:
            {
                target_angle = target_config.ladder_touch;
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

            pros::delay(20);
        } });
}

void mechanism::Arm::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Arm::set_state(mechanism::ArmState state)
{
    mutex.lock();
    if (state == ArmState::DISABLED)
    {
        motors->move(0);
    }

    this->state = state;
    mutex.unlock();
}

mechanism::ArmState mechanism::Arm::get_state()
{
    mutex.lock();
    auto temp = state;
    mutex.unlock();

    return temp;
}

bool mechanism::Arm::is_loading()
{
    double current = 360.0 - (arm_rotation_sensor->get_angle() / 100.0);

    if (current > 180.0)
    {
        current = 0.0;
    }

    return std::abs(current - target_config.load) < 10.0;
}

void mechanism::Arm::move_manual(double voltage)
{
    mutex.lock();
    state = ArmState::DISABLED;

    motors->move(voltage);
    mutex.unlock();
}