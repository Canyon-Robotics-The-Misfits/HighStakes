#include "intake.h"
#include "lib15442c/logger.hpp"
#include "pros/misc.h"

#define LOGGER "intake.cpp"

mechanism::Intake::Intake(std::shared_ptr<lib15442c::MotorGroup> motors, std::shared_ptr<lib15442c::Pneumatic> redirect)
    : motors(motors),
      redirect(redirect)
{
    if (!motors->is_installed())
    {
        ERROR_TEXT("Intake motor(s) unplugged!");
    }

    start_task();
}

void mechanism::Intake::start_task()
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
            
            // TODO: make intake work

            pros::delay(20);
        } });
}

void mechanism::Intake::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Intake::set_state(mechanism::IntakeState state)
{
    mutex.lock();
    if (state == DISABLED)
    {
        motors->move(0);
    }

    this->state = state;
    mutex.unlock();
}

mechanism::IntakeState mechanism::Intake::get_state()
{
    mutex.lock();
    auto temp = state;
    mutex.unlock();

    return temp;
}