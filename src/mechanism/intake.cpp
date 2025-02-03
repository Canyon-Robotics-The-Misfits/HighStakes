#include "intake.h"
#include "lib15442c/logger.hpp"
#include "pros/misc.h"

#include <iostream>

#define LOGGER "intake.cpp"

mechanism::Intake::Intake(std::shared_ptr<lib15442c::IMotor> motors, std::shared_ptr<lib15442c::Pneumatic> redirect)
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

    redirect->extend();

    task = pros::Task([this]()
                      {
        auto initial_comp_status = pros::c::competition_get_status();

        int i = 0;
        
        while (pros::c::competition_get_status() == initial_comp_status)
        {
            mutex.lock();
            if (task_on_flag == false)
            {
                mutex.unlock();
                break;
            }

            // if (this->state != IntakeState::DISABLED && this->state != IntakeState::DEJAM && std::abs(motors->get_velocity()) < 300)
            // {
            //     this->state = IntakeState::DEJAM;
            // }

            IntakeState state = this->state;

            mutex.unlock();

            
            switch (state)
            {
            case IntakeState::HOOD: {
                // std::cout << "hood" << std::endl;
                motors->move(127 - (i % 20 == 0 ? 1 : 0));
                redirect->extend();
            } break;
            case IntakeState::WALL_STAKE: {
                // std::cout << "wall" << std::endl;
                motors->move(127 - (i % 20 == 0 ? 1 : 0));
                redirect->retract();
            } break;
            case IntakeState::REVERSE: {
                // std::cout << "rev" << std::endl;
                motors->move(-127 + (i % 20 == 0 ? 1 : 0));
                redirect->extend();
            } break;
            case IntakeState::DEJAM: {
                // std::cout << "dejam" << std::endl;
                motors->move(-127 + (i % 20 == 0 ? 1 : 0));
                // redirect->extend();
            } break;
            case IntakeState::DISABLED: {
                // std::cout << "disable" << std::endl;
                motors->move(0);
            } break;
            }

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
    if (state == IntakeState::DISABLED)
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