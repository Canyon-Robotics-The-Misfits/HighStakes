#pragma once

#include <memory>

#include "arm.hpp"
#include "gui/gui.h"

#include "lib15442c/device/pneumatic.hpp"
#include "lib15442c/chasis/tank_drive.hpp"
#include "pros/optical.hpp"

#include "lib15442c/math/angle.hpp"

namespace mechanism
{
    constexpr double LB_IDLE_ANGLE_DEG = -106;
    constexpr double LB_LOAD_ANGLE_DEG = -85;
    constexpr double LB_SCORE_ANGLE_DEG = 67;
    constexpr double LB_SCORE_SKILLS_ANGLE_DEG = 33;
    constexpr double LB_DESCORE_1_ANGLE_DEG = 42;
    constexpr double LB_DESCORE_2_ANGLE_DEG = 45;
    constexpr double LB_CLIMB_PREP_ANGLE_DEG = -50;

    enum class RingManagerState
    {
        IDLE,
        INTAKE,
        INTAKE_OVERRIDE,
        INTAKE_REVERSE,
        INTAKE_HOLD,
        LOAD,
        HOLD,
        SCORE,
        SCORE_SKILLS,
        DESCORE_1,
        DESCORE_2,
        PREP_CLIMB,
        CLIMBING
    };

    enum class SortColor
    {
        NONE,
        RED,
        BLUE
    };

    class RingManager
    {
    private:
        std::shared_ptr<mechanism::Arm> lb;
        std::shared_ptr<lib15442c::IMotor> intake_motors;

        std::shared_ptr<pros::Optical> optical_sensor;

        std::shared_ptr<lib15442c::IPneumatic> lb_lift_push;
        std::shared_ptr<lib15442c::IPneumatic> lb_lift_pull;

        std::shared_ptr<lib15442c::IPneumatic> pto;
        std::shared_ptr<lib15442c::TankDrive> drivetrain;

        RingManagerState current_state = RingManagerState::IDLE;
        bool lb_override = false;
        SortColor sort_color = SortColor::NONE;

        double sort_countdown = 0;

        void update_devices();
        void run_dejam();
        void run_color_sort();
        void set_state(RingManagerState state);

        void climb_macro(int tier);
        
        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]() { return; });

    public:
        RingManager(
            std::shared_ptr<mechanism::Arm> lb, std::shared_ptr<lib15442c::IMotor> intake_motors,
            std::shared_ptr<pros::Optical> optical_sensor, std::shared_ptr<lib15442c::IPneumatic> lb_lift_push,
            std::shared_ptr<lib15442c::IPneumatic> lb_lift_pull, std::shared_ptr<lib15442c::IPneumatic> pto,
            std::shared_ptr<lib15442c::TankDrive> drivetrain
        );

        /**
         * @brief Start the task
         */
        void start_task();
        /**
         * @brief Stop the task
         */
        void stop_task();

        void intake();
        void intake_override();
        void intake_reverse();
        void stop_intake();
        
        void intake_hold();
        
        void load();
        void stop_load();
        
        void descore_1();
        void descore_2();

        void score();
        void score_skills();
        void idle();

        void prep_climb();
        void climb(int tier = 3);

        void set_lb_override(bool lb_overrided);

        void set_color_sort(SortColor color);
        
        mechanism::RingManagerState get_state();
        bool ring_detected();
    };
} // namespace mechanism
