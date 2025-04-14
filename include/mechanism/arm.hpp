#pragma once

#include <memory>
#include "pros/rtos.hpp"

#include "lib15442c/math/angle.hpp"
#include "lib15442c/device/motor.hpp"
#include "pros/rotation.hpp"

#include "lib15442c/controller/pid.hpp"

namespace mechanism
{
    class Arm
    {
    private:
        std::shared_ptr<lib15442c::IMotor> motors;
        std::shared_ptr<pros::Rotation> rotation_sensor;

        std::shared_ptr<lib15442c::PID> pid;
        double kG;

        lib15442c::Angle target_angle = lib15442c::Angle::none();
        double voltage_override = 0;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]() { return; });
    
    public:
        Arm(std::shared_ptr<lib15442c::IMotor> motors, std::shared_ptr<pros::Rotation> rotation_sensor, std::shared_ptr<lib15442c::PID> pid, double kG);
        
        /**
         * @brief Start the task
         */
        void start_task();
        /**
         * @brief Stop the task
         */
        void stop_task();

        /**
         * @brief Set the arm's target angle
         * 
         * @param target The target angle
         */
        void set_target(lib15442c::Angle target);

        /**
         * @brief Whether the arm has reached it's target
         * 
         * @return bool
         */
        bool is_settled(double threshold_deg = 5);

        /**
         * @brief Manually move the arm. Automatically resets target angle
         * 
         * @param voltage the voltage to move at
         */
        void move(double voltage);

        /**
         * @brief Get the current angle of the arm
         * 
         * @return lib15442c::Angle 
         */
        lib15442c::Angle get_current_angle();
    };
} // namespace mechanism
