#pragma once

#include <memory>
#include <variant>

#include "lib15442c/device/motor.hpp"
#include "lib15442c/controller/pid.hpp"
#include "pros/rtos.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"

namespace mechanism
{
    struct ArmTargetConfig
    {
        double load;
        double color_sort;
        double alliance_stake;
        double neutral_stake;
    };

    enum class ArmTarget
    {
        MANUAL,
        LOAD,
        COLOR_SORT,
        ALLIANCE_STAKE,
        NEUTRAL_STAKE
    };

    class Arm
    {
    private:
        std::shared_ptr<lib15442c::Motor> motor;
        std::shared_ptr<pros::Rotation> rotation_sensor;
        std::shared_ptr<pros::adi::DigitalIn> limit_switch;
        std::shared_ptr<lib15442c::PID> pid;

        ArmTarget target = ArmTarget::LOAD;

        ArmTargetConfig target_config;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]() { return; });

    public:
        Arm(std::shared_ptr<lib15442c::Motor> motor, std::shared_ptr<pros::Rotation> rotation_sensor, std::shared_ptr<pros::adi::DigitalIn> limit_switch, std::shared_ptr<lib15442c::PID> pid, ArmTargetConfig target_config);

        /**
         * @brief Start the color sort task
         */
        void start_task();
        /**
         * @brief Stop the color sort task
         */
        void stop_task();
        
        /**
         * @brief Move the arm manually (automatically sets arm to manual mode)
         * 
         * @param voltage The voltage to move at
         */
        void move(double voltage);

        /**
         * @brief Set the position the arm should target
         * 
         * @param target The new target
         */
        void set_target(ArmTarget target);

        /**
         * @brief Get what position the arm is currently targeting
         * 
         * @return ArmTarget The current target
         */
        ArmTarget get_target();

    };
} // namespace mechanism