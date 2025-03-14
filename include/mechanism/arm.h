#pragma once

#include <memory>
#include "pros/rtos.hpp"

#include "lib15442c/device/motor.hpp"

#include "lib15442c/controller/pid.hpp"
#include "pros/rotation.hpp"

#include "intake.h"

namespace mechanism
{
    struct ArmTargetConfig
    {
        double load;
        double alliance_stake;
        double ladder_touch;
        double neutral_stake;
        double climb;
    };

    enum class ArmState
    {
        DISABLED,
        LOAD,
        ALLIANCE_STAKE,
        LADDER_TOUCH,
        NEUTRAL_STAKE,
        CLIMB
    };

    class Arm
    {
    private:
        std::shared_ptr<lib15442c::IMotor> motors;
        std::shared_ptr<pros::Rotation> arm_rotation_sensor;

        std::shared_ptr<mechanism::Intake> intake;

        std::shared_ptr<lib15442c::PID> arm_pid;

        ArmTargetConfig target_config;
        double kG;

        ArmState state = ArmState::DISABLED;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]() { return; });

    public:
        Arm(std::shared_ptr<lib15442c::IMotor> motors, std::shared_ptr<pros::Rotation> arm_rotation_sensor, std::shared_ptr<mechanism::Intake> intake, std::shared_ptr<lib15442c::PID> arm_pid, ArmTargetConfig arm_target_config, double kG);
        
        /**
         * @brief Start the task
         */
        void start_task();
        /**
         * @brief Stop the task
         */
        void stop_task();

        /**
         * @brief Set the state of the mechanism
         * 
         * @param state The new state
         */
        void set_state(ArmState state);

        /**
         * @brief Get the state of the mechanism
         * 
         * @return RingMechState The current state
         */
        ArmState get_state();

        /**
         * @brief Whether the arm is in loading position
         * 
         * @return bool
         */
        bool is_loading();

        /**
         * @brief Manually move the arm. Automatically sets state to DISABLED
         * 
         * @param voltage the voltage to move at
         */
        void move_manual(double voltage);
    };
} // namespace mechanism
