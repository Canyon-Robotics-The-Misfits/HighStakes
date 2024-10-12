#pragma once

#include <memory>

#include "lib15442c/device/motor.hpp"

#include "lib15442c/device/pneumatic.hpp"
#include "pros/optical.hpp"

#include "lib15442c/controller/pid.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"

#include <variant>

namespace mechanism
{
    struct ArmTargetConfig
    {
        double load;
        double alliance_stake;
        double ladder_touch;
        double neutral_stake;
    };

    enum RingMechState
    {
        DISABLED = 0x00000,

        ARM_LOAD = 0x10000,
        ARM_ALLIANCE_STAKE = 0x10001,
        ARM_NEUTRAL_STAKE = 0x10010,
        ARM_LADDER_TOUCH = 0x10011,

        INTAKE_HOOD = 0x11000,
        INTAKE_WALL_STAKE = 0x11001,
        INTAKE_OUTTAKE = 0x11010,
        INTAKE_SORT_BLUE = 0x11011,
        INTAKE_SORT_RED = 0x11100,
    };

    struct RingMechParams
    {
        std::shared_ptr<lib15442c::MotorGroup> motors;

        std::shared_ptr<lib15442c::IPneumatic> intake_redirect;
        std::shared_ptr<pros::Optical> intake_optical;
        
        std::shared_ptr<pros::Rotation> arm_rotation_sensor;
        std::shared_ptr<pros::adi::DigitalIn> arm_limit;
        std::shared_ptr<lib15442c::PID> arm_pid; 
    };

    class RingMech
    {
    private:
        std::shared_ptr<lib15442c::MotorGroup> motors;

        std::shared_ptr<lib15442c::IPneumatic> intake_redirect;
        std::shared_ptr<pros::Optical> intake_optical;
        
        std::shared_ptr<pros::Rotation> arm_rotation_sensor;
        std::shared_ptr<pros::adi::DigitalIn> arm_limit;
        std::shared_ptr<lib15442c::PID> arm_pid;

        ArmTargetConfig arm_target_config;

        RingMechState state = RingMechState::DISABLED;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]() { return; });

        bool is_intake();
        bool is_arm();

    public:
        RingMech(RingMechParams params);
        
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
        void set_state(RingMechState state);

        /**
         * @brief Get the state of the mechanism
         * 
         * @return RingMechState The current state
         */
        RingMechState get_state();

        bool is_arm_loading();
    };
} // namespace mechanism
