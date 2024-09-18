#pragma once

#include "lib15442c/device/motor.hpp"
#include "lib15442c/device/pneumatic.hpp"
#include "pros/optical.hpp"

namespace mechanism
{
    enum class IntakeRedirectMode
    {
        /**
         * @brief Send all rings to the hood
         */
        NONE,
        /**
         * @brief Color sort and redirect red, only allow blue rings to the hood
         */
        RED,
        /**
         * @brief Color sort and redirect blue, only allow red rings to the hood
         */
        BLUE,
        /**
         * @brief Redirect every ring, sending none to the hood
         */
        ALL
    };

    class Intake
    {
    private:
        std::shared_ptr<lib15442c::Motor> motor;
        std::shared_ptr<lib15442c::IPneumatic> redirect;
        std::shared_ptr<pros::Optical> color_sensor;

        IntakeRedirectMode redirect_mode = IntakeRedirectMode::NONE;

        bool task_on_flag = false;
        pros::Mutex mutex;
        pros::Task task = pros::Task([]() { return; });

    public:
        Intake(std::shared_ptr<lib15442c::Motor> motor, std::shared_ptr<lib15442c::IPneumatic> redirect, std::shared_ptr<pros::Optical> color_sensor);

        /**
         * @brief Start the color sort task
         */
        void start_task();
        /**
         * @brief Stop the color sort task
         */
        void stop_task();

        /**
         * @brief Move the intake motor at a set voltage
         * 
         * @param voltage The voltage to move at
         */
        void move(double voltage);

        /**
         * @brief Set the mode of the auto-redirect
         * 
         * @param redirect_mode The mode to use
         */
        void set_redirect_mode(IntakeRedirectMode redirect_mode);

        /**
         * @brief Get the mode of the auto-redirect
         * 
         * @return IntakeRedirectMode The mode being used
         */
        IntakeRedirectMode get_redirect_mode();
    };
}