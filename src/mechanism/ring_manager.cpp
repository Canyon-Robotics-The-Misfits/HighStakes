#include "ring_manager.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "ring_manager.cpp"

mechanism::RingManager::RingManager(
    std::shared_ptr<mechanism::Arm> lb, std::shared_ptr<lib15442c::IMotor> intake_motors,
    std::shared_ptr<pros::Optical> optical_sensor, std::shared_ptr<lib15442c::IPneumatic> lb_lift_push,
    std::shared_ptr<lib15442c::IPneumatic> lb_lift_pull, std::shared_ptr<lib15442c::IPneumatic> pto,
    std::shared_ptr<lib15442c::TankDrive> drivetrain
)
    : lb(lb), intake_motors(intake_motors), optical_sensor(optical_sensor), lb_lift_push(lb_lift_push), lb_lift_pull(lb_lift_pull), pto(pto), drivetrain(drivetrain)
{
    if (!intake_motors->is_installed())
    {
        ERROR_TEXT("Intake motor(s) unplugged!");
    }
    if (!optical_sensor->is_installed())
    {
        ERROR("Color sensor is not detected on port %d!", optical_sensor->get_port());
    }
    optical_sensor->set_led_pwm(100);

    start_task();
}

void mechanism::RingManager::start_task()
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
        auto initial_comp_status = pros::c::competition_get_status();
        
        while (initial_comp_status == pros::c::competition_get_status())
        {
            mutex.lock();
            if (task_on_flag == false)
            {
                mutex.unlock();
                break;
            }
            mutex.unlock();
            
            update_devices();

            pros::delay(20);
        } });
}

void mechanism::RingManager::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}


// idle = intake off, lb idle
// intake = intake spins, lb idle ; go to idle when stopped
// intake reverse = intake spins reverse, lb unaffected ; go to idle when stopped
// load = intake spins (while lb settled), lb load ; go to hold when stopped
// hold = intake off, lb load
// score = intake off, lb score (when settle go to idle)
// climb = run climb macro
void mechanism::RingManager::update_devices()
{
    static lib15442c::Angle lb_idle = lib15442c::Angle::from_deg(LB_IDLE_ANGLE_DEG);
    static lib15442c::Angle lb_load = lib15442c::Angle::from_deg(LB_LOAD_ANGLE_DEG);
    static lib15442c::Angle lb_score = lib15442c::Angle::from_deg(LB_SCORE_ANGLE_DEG);
    static lib15442c::Angle lb_score_skills = lib15442c::Angle::from_deg(LB_SCORE_SKILLS_ANGLE_DEG);
    
    static lib15442c::Angle lb_descore_1 = lib15442c::Angle::from_deg(LB_DESCORE_1_ANGLE_DEG);
    static lib15442c::Angle lb_descore_2 = lib15442c::Angle::from_deg(LB_DESCORE_2_ANGLE_DEG);
    
    static lib15442c::Angle lb_climb_prep = lib15442c::Angle::from_deg(LB_CLIMB_PREP_ANGLE_DEG);

    mutex.lock();

    switch (current_state)
    {

    case RingManagerState::IDLE: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_idle);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::INTAKE: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
        
        if (!lb_override)
        {
            lb->set_target(lb_idle);
        }

        if (lb->is_settled())
        {
            run_color_sort();
        }
        else
        {
            intake_motors->move(0);
        }
    } break;

    case RingManagerState::INTAKE_OVERRIDE: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        intake_motors->move(127);
    } break;

    case RingManagerState::INTAKE_REVERSE: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        intake_motors->move(-127);
    } break;

    case RingManagerState::INTAKE_HOLD: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::HOLD);
        
        intake_motors->move(0);
    } break;

    case RingManagerState::LOAD: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_load);
        }

        if (lb->is_settled())
        {
            intake_motors->move(127);
        }
        else
        {
            intake_motors->move(0);
        }
    } break;

    case RingManagerState::HOLD: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_load);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::SCORE: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_score);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::SCORE_SKILLS: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_score_skills);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::DESCORE_1: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_descore_1);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::DESCORE_2: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        if (!lb_override)
        {
            lb->set_target(lb_descore_2);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::PREP_CLIMB: {
        intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

        lb_lift_pull->retract();
        lb_lift_push->extend();
        lb->set_target(lb_climb_prep);
        intake_motors->move(0);
    } break;

    case RingManagerState::CLIMBING: {
        // nothing, managed in climb_macro()
    } break;

    }

    mutex.unlock();
}

void mechanism::RingManager::run_dejam()
{
    if (intake_motors->get_power() > 10)
    {
        intake_motors->move(-60);
    }
    else
    {
        intake_motors->move(127);
    }
}

// int i = 0;
void mechanism::RingManager::run_color_sort()
{
    // std::cout << i << ", " << optical_sensor->get_proximity() << ", " << optical_sensor->get_hue() << std::endl;
    // i++;

    if (ring_detected() && sort_countdown <= 0)
    {
        double hue = optical_sensor->get_hue(); 
    
        bool is_blue = hue > 65 && hue < 250;
        bool is_red = hue < 35 || hue > 250;

        if ((is_red && sort_color == SortColor::RED) || (is_blue && sort_color == SortColor::BLUE))
        {
            sort_countdown = 400;
        }
    }

    if (sort_countdown > 0)
    {
        sort_countdown -= 20;

        if (sort_countdown > 300)
        {
            run_dejam();
        }
        else
        {
            intake_motors->move(-127);
        }
    }
    else
    {
        intake_motors->move(127);
    }
}

mechanism::RingManagerState mechanism::RingManager::get_state()
{
    mutex.lock();
    auto temp = current_state;
    mutex.unlock();
    return temp;
}

void mechanism::RingManager::set_state(RingManagerState state)
{
    mutex.lock();
    current_state = state;
    mutex.unlock();
}

#define WAIT_UNTIL(condition) while (!(condition)) { pros::delay(10); }

void mechanism::RingManager::climb_macro(int tier)
{
    using namespace lib15442c::literals;
    
    mutex.lock();

    int start_time = pros::millis();

    intake_motors->set_brake_mode(lib15442c::MotorBrakeMode::HOLD);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

    pto->extend();
    lb->move(-127);

    WAIT_UNTIL(lb->get_current_angle().deg() < -55);
    
    drivetrain->move(80, 0);

    WAIT_UNTIL(lb->get_current_angle().deg() < -67);
    
    drivetrain->move(127, 0);

    for (int i = 0; i < std::min(tier, 3); i++)
    {
        pto->extend();
        lb->move(-127);

        pros::delay(100);

        lb_lift_push->retract();
        lb_lift_pull->extend();

        pros::delay(100);

        drivetrain->move(127, 0);
    
        WAIT_UNTIL(lb->get_current_angle().deg() < -113);

        pros::delay(150);

        if (i == tier - 1 && i != 2)
        {
            drivetrain->move(0, 0);
            lb->move(0);

            break;
        }
    
        drivetrain->move(0, 0);
        lb->move(0);
    
        lb->set_target(2_deg);
        
        drivetrain->move(-127, 0);
        pros::delay(100);
        pto->retract();
        pros::delay(250);
        drivetrain->move(0, 0);
        
        if (i == 2)
        {
            pros::delay(100); // meditation break

            intake_motors->move(127);
            pros::delay(700);
            intake_motors->move(0);
            
            drivetrain->move(0, 0);

            break;
        }
    
        WAIT_UNTIL(lb->get_current_angle().deg() > -17);
    
        lb_lift_push->extend();
        lb_lift_pull->retract();
    
        pros::delay(300);
    }

    int end_time = pros::millis();

    std::cout << "Climb time: " << end_time - start_time << "ms" << std::endl;

    drivetrain->move(0, 0);
    lb->move(0);
    
    mutex.unlock();
}

void mechanism::RingManager::intake()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }
    
    set_state(RingManagerState::INTAKE);
}
void mechanism::RingManager::intake_override()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::INTAKE_OVERRIDE);
}
void mechanism::RingManager::intake_reverse()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::INTAKE_REVERSE);
}
void mechanism::RingManager::stop_intake()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::IDLE);
}
void mechanism::RingManager::intake_hold()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::INTAKE_HOLD);
}

void mechanism::RingManager::load()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::LOAD);
}
void mechanism::RingManager::stop_load()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::HOLD);
}

void mechanism::RingManager::descore_1()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::DESCORE_1);
}
void mechanism::RingManager::descore_2()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::DESCORE_2);
}

void mechanism::RingManager::score()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::SCORE);
}

void mechanism::RingManager::score_skills()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        return;
    }

    set_state(RingManagerState::SCORE_SKILLS);
}

void mechanism::RingManager::idle()
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        lb_lift_pull->retract();
        lb_lift_push->retract();
    }

    set_state(RingManagerState::IDLE);
}

void mechanism::RingManager::prep_climb()
{
    set_state(RingManagerState::PREP_CLIMB);
}

void mechanism::RingManager::climb(int tier)
{
    if (get_state() == RingManagerState::PREP_CLIMB)
    {
        set_state(RingManagerState::CLIMBING);
        climb_macro(tier);
    }
}

void mechanism::RingManager::set_lb_override(bool lb_override)
{
    mutex.lock();
    this->lb_override = lb_override;
    mutex.unlock();
}


void mechanism::RingManager::set_color_sort(SortColor color)
{
    mutex.lock();
    sort_color = color;
    mutex.unlock();
}


bool mechanism::RingManager::ring_detected()
{
    return optical_sensor->get_proximity() > 50;
}
