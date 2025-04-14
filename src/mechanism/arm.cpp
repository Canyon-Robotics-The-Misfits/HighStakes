#include "arm.hpp"
#include "lib15442c/logger.hpp"
#include "pros/misc.h"

#define LOGGER "arm.cpp"

mechanism::Arm::Arm(std::shared_ptr<lib15442c::IMotor> motors, std::shared_ptr<pros::Rotation> rotation_sensor, std::shared_ptr<lib15442c::PID> pid, double kG)
    : motors(motors),
      rotation_sensor(rotation_sensor),
      pid(pid),
      kG(kG)
{
    if (!motors->is_installed())
    {
        ERROR_TEXT("Arm motor(s) unplugged!");
    }
    if (!rotation_sensor->is_installed())
    {
        ERROR("Arm rotation sensor is not detected on port %d!", rotation_sensor->get_port());
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
        auto initial_comp_status = pros::c::competition_get_status();
        
        while (initial_comp_status == pros::c::competition_get_status())
        {
            mutex.lock();
            if (task_on_flag == false)
            {
                break;
            }
            mutex.unlock();
            
            lib15442c::Angle current_angle = get_current_angle();

            double pwm;
            if (!target_angle.is_none())
            {
                double error = current_angle.error_from(target_angle).deg();
    
                pwm = pid->calculate_error(error);

            }
            else if (voltage_override != 0)
            {
                pwm = voltage_override;
            }
            else
            {
                pwm = 0;
            }
            
            motors->move(pwm - kG * sin(current_angle.rad_unwrapped()));

            pros::delay(20);
        } });
}

void mechanism::Arm::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Arm::set_target(lib15442c::Angle target)
{
    mutex.lock();
    target_angle = target;
    mutex.unlock();
}

bool mechanism::Arm::is_settled(double threshold_deg)
{
    return abs((get_current_angle() - target_angle).deg()) < threshold_deg;
}

void mechanism::Arm::move(double voltage)
{
    mutex.lock();
    target_angle = lib15442c::Angle::none();
    voltage_override = voltage;
    mutex.unlock();
}

lib15442c::Angle mechanism::Arm::get_current_angle()
{
    return lib15442c::Angle::from_deg((-rotation_sensor->get_position() / 100.0) / 5.0);
}