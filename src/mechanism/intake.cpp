#include "intake.h"
#include "lib15442c/logger.hpp"

#define LOGGER "intake.cpp"

mechanism::Intake::Intake(std::shared_ptr<lib15442c::Motor> motor, std::shared_ptr<lib15442c::IPneumatic> redirect, std::shared_ptr<pros::Optical> color_sensor)
    : motor(motor), redirect(redirect), color_sensor(color_sensor)
{
    if (!motor->is_installed())
    {
		ERROR("Intake motor is not detected on port %d!", motor->get_port());
    }
    if (!color_sensor->is_installed())
    {
		ERROR("Optical sensor is not detected on port %d!", color_sensor->get_port());
    }

    color_sensor->set_led_pwm(100);

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
            if (!task_on_flag)
            {
                mutex.unlock();
                break;
            }

            double hue = color_sensor->get_hue();

            bool red = (hue > 0 && hue < 30) || (hue > 350 && hue < 360);
            bool blue = hue > 170 && hue < 250;

            // if (red) printf("red\n");
            // if (blue) printf("blue\n");

            switch (redirect_mode)
            {
                case IntakeRedirectMode::NONE: {
                    redirect->extend();
                } break;
                case IntakeRedirectMode::RED: {
                    if (red)
                    {
                        redirect->retract();
                    }
                    else if (blue)
                    {
                        redirect->extend();
                    }
                } break;
                case IntakeRedirectMode::BLUE: {
                    if (blue)
                    {
                        redirect->retract();
                    }
                    else if (red)
                    {
                        redirect->extend();
                    }
                } break;
                case IntakeRedirectMode::ALL: {
                    redirect->retract();
                } break;
            }

            mutex.unlock();

            pros::delay(20);
        }
    });
}

void mechanism::Intake::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Intake::move(double voltage)
{
    mutex.lock();
    motor->move(voltage);
    mutex.unlock();
}

void mechanism::Intake::set_redirect_mode(IntakeRedirectMode mode)
{
    mutex.lock();
    redirect_mode = mode;

    // if (redirect_mode == IntakeRedirectMode::NONE)
    // {
    //     redirect->extend();
    // }
    // else if (redirect_mode == IntakeRedirectMode::ALL)
    // {
    //     redirect->retract();
    // }

    mutex.unlock();
}

mechanism::IntakeRedirectMode mechanism::Intake::get_redirect_mode()
{
    mutex.lock();
    IntakeRedirectMode mode = redirect_mode;
    mutex.unlock();

    return mode;
}