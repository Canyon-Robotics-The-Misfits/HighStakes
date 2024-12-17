#pragma once

#include <memory>
#include "pros/rtos.hpp"

#include "lib15442c/device/motor.hpp"
#include "lib15442c/device/pneumatic.hpp"

#include "lib15442c/controller/pid.hpp"

namespace mechanism
{
    enum IntakeState
    {
        DISABLED,
        HOOD,
        WALL_STAKE,
        REVERSE,
        DEJAM
    };

    class Intake
    {
    private:
        std::shared_ptr<lib15442c::MotorGroup> motors;

        std::shared_ptr<lib15442c::Pneumatic> redirect;

        IntakeState state = IntakeState::DISABLED;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]() { return; });

    public:
        Intake(std::shared_ptr<lib15442c::MotorGroup> motors, std::shared_ptr<lib15442c::Pneumatic> redirect);
        
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
        void set_state(IntakeState state);

        /**
         * @brief Get the state of the mechanism
         * 
         * @return RingMechState The current state
         */
        IntakeState get_state();
    };
} // namespace mechanism
