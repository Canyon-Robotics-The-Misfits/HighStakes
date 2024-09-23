#include "arm.h"
#include "lib15442c/logger.hpp"

#define LOGGER "arm.cpp"

mechanism::Arm::Arm(std::shared_ptr<lib15442c::Motor> motor, std::shared_ptr<pros::Rotation> rotation_sensor, std::shared_ptr<pros::adi::DigitalIn> limit_switch, std::shared_ptr<lib15442c::PID> pid, ArmTargetConfig target_config)
    : motor(motor), rotation_sensor(rotation_sensor), limit_switch(limit_switch), pid(pid), target_config(target_config)
{
    if (!motor->is_installed())
    {
		ERROR("Arm motor is not detected on port %d!", motor->get_port());
    }
    if (!rotation_sensor->is_installed())
    {
		ERROR("Rotation sensor is not detected on port %d!", rotation_sensor->get_port());
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
            if (!task_on_flag)
            {
                mutex.unlock();
                break;
            }

            double current_angle = rotation_sensor->get_angle() / 100.0; // divide by 100 to convert centidegrees to degrees

            // make 359 equal to 0
            if (current_angle > 180)
            {
                current_angle = 0;
            }

            double target_angle;

            switch (target)
            {
            case ArmTarget::LOAD: {
                target_angle = target_config.load;
            } break;
            case ArmTarget::COLOR_SORT: {
                target_angle = target_config.color_sort;
            } break;
            case ArmTarget::ALLIANCE_STAKE: {
                target_angle = target_config.alliance_stake;
            } break;
            case ArmTarget::NEUTRAL_STAKE: {
                target_angle = target_config.neutral_stake;
            } break;
            default: {
                target_angle = INFINITY;
            };
            }

            if (target != ArmTarget::MANUAL)
            {
                double output = pid->calculate(current_angle, target_angle);

                if (limit_switch->get_value() == true)
                {
                    output = std::max(output, 0.0);
                }

                motor->move(output);
            }

            mutex.unlock();

            pros::delay(20);
        } });
}

void mechanism::Arm::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Arm::move(double voltage)
{
    if (voltage != 0.0)
    {
        set_target(ArmTarget::MANUAL);
    }

    mutex.lock();
    if (limit_switch->get_value() == true)
    {
        voltage = std::max(voltage, 0.0);
    }

    motor->move(voltage);
    mutex.unlock();
}

void mechanism::Arm::set_target(ArmTarget target)
{
    mutex.lock();
    this->target = target;
    mutex.unlock();
}

mechanism::ArmTarget mechanism::Arm::get_target()
{
    mutex.lock();
    auto temp = target;
    mutex.unlock();

    return temp;
}