#include "ring_manager.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "ring_manager.cpp"

mechanism::RingManager::RingManager(std::shared_ptr<mechanism::Arm> lb, std::shared_ptr<lib15442c::IMotor> intake_motors, std::shared_ptr<pros::Optical> optical_sensor, std::shared_ptr<lib15442c::IPneumatic> lb_lift)
    : lb(lb), intake_motors(intake_motors), optical_sensor(optical_sensor), lb_lift(lb_lift)
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

    mutex.lock();

    switch (current_state)
    {

    case RingManagerState::IDLE: {
        if (!lb_override)
        {
            lb->set_target(lb_idle);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::INTAKE: {
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
        intake_motors->move(127);
    } break;

    case RingManagerState::INTAKE_REVERSE: {
        intake_motors->move(-127);
    } break;

    case RingManagerState::LOAD: {
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
        if (!lb_override)
        {
            lb->set_target(lb_load);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::SCORE: {
        if (!lb_override)
        {
            lb->set_target(lb_score);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::SCORE_SKILLS: {
        if (!lb_override)
        {
            lb->set_target(lb_score_skills);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::DESCORE_1: {
        if (!lb_override)
        {
            lb->set_target(lb_descore_1);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::DESCORE_2: {
        if (!lb_override)
        {
            lb->set_target(lb_descore_2);
        }
        intake_motors->move(0);
    } break;

    case RingManagerState::CLIMBING: {
        // TODO
    } break;

    }

    mutex.unlock();
}

// int i = 0;
void mechanism::RingManager::run_color_sort()
{
    // std::cout << i << ", " << optical_sensor->get_proximity() << ", " << optical_sensor->get_hue() << std::endl;
    // i++;

    if (optical_sensor->get_proximity() > 50 && sort_countdown <= 0)
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
            intake_motors->move(127);
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

void mechanism::RingManager::set_state(RingManagerState state)
{
    mutex.lock();
    current_state = state;
    mutex.unlock();
}

void mechanism::RingManager::intake()
{
    set_state(RingManagerState::INTAKE);
}
void mechanism::RingManager::intake_override()
{
    set_state(RingManagerState::INTAKE_OVERRIDE);
}
void mechanism::RingManager::intake_reverse()
{
    set_state(RingManagerState::INTAKE_REVERSE);
}
void mechanism::RingManager::stop_intake()
{
    set_state(RingManagerState::IDLE);
}

void mechanism::RingManager::load()
{
    set_state(RingManagerState::LOAD);
}
void mechanism::RingManager::stop_load()
{
    set_state(RingManagerState::HOLD);
}

void mechanism::RingManager::descore_1()
{
    set_state(RingManagerState::DESCORE_1);
}
void mechanism::RingManager::descore_2()
{
    set_state(RingManagerState::DESCORE_2);
}

void mechanism::RingManager::score()
{
    set_state(RingManagerState::SCORE);
}

void mechanism::RingManager::score_skills()
{
    set_state(RingManagerState::SCORE_SKILLS);
}

void mechanism::RingManager::idle()
{
    set_state(RingManagerState::IDLE);
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