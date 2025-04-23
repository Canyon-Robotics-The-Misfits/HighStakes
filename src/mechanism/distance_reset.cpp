#include "distance_reset.hpp"

#define LOGGER "distance_reset.cpp"

bool check_conditions(pros::Distance sensor, std::string name)
{
    if (!sensor.is_installed())
    {
        ERROR("%s distance sensor not detedted on port %d!", name.c_str(), sensor.get_port());
        return false;
    }
    if (sensor.get() == 9999)
    {
        WARN("%s distance sensor is out of range!", name.c_str());
        return false;
    }
    if (sensor.get() * mechanism::MM_TO_IN * 0.05 > 2)
    {
        WARN("%s distance sensor has an error of >2!", name.c_str());
        return false;
    }
    else
    {
        return true;
    }
}

void mechanism::distance_reset(std::shared_ptr<lib15442c::TrackerOdom> odometry, bool disable_left, bool disable_right, bool disable_front)
{
    lib15442c::Vec initial_pos = odometry->get_position();

    pros::Distance left_sensor = pros::Distance(config::PORT_DISTANCE_LEFT);
    pros::Distance right_sensor = pros::Distance(config::PORT_DISTANCE_RIGHT);
    pros::Distance front_sensor = pros::Distance(config::PORT_DISTANCE_FRONT);

    bool left_passed = !disable_left && check_conditions(left_sensor, "left");
    bool right_passed = !disable_right && check_conditions(right_sensor, "right");
    bool front_passed = !disable_front && check_conditions(front_sensor, "front");

    double left = left_sensor.get() * MM_TO_IN + LEFT_DISTANCE_OFFSET;
    double right = right_sensor.get() * MM_TO_IN + RIGHT_DISTANCE_OFFSET;
    double front = front_sensor.get() * MM_TO_IN + FRONT_DISTANCE_OFFSET;

    lib15442c::Angle heading = odometry->get_rotation();

    if (!(left_passed || right_passed))
    {
        WARN_TEXT("No horrizontal sensor in range for reset!");
    }
    
    if (!front_passed)
    {
        WARN_TEXT("Front sensor out of range for reset!");
    }

    if (abs(heading.deg()) < RESET_THRESHOLD_DEG)
    {
        if (front_passed)
        {
            odometry->set_y(144 - front);
        }

        if (left < right && left_passed)
        {
            odometry->set_x(left);
        }
        else if (right_passed)
        {
            odometry->set_x(144 - right);
        }
    }
    else if (abs((heading - 90_deg).deg()) < RESET_THRESHOLD_DEG)
    {
        if (left < right && left_passed)
        {
            odometry->set_y(144 - left);
        }
        else if (right_passed)
        {
            odometry->set_y(right);
        }
        
        if (front_passed)
        {
            odometry->set_x(144 - front);
        }
    }
    else if (abs((heading - 180_deg).deg()) < RESET_THRESHOLD_DEG)
    {
        if (front_passed)
        {
            odometry->set_y(front);
        }

        if (left < right && left_passed)
        {
            odometry->set_x(144 - left);
        }
        else if (right_passed)
        {
            odometry->set_x(right);
        }
    }
    else if (abs((heading - 270_deg).deg()) < RESET_THRESHOLD_DEG)
    {
        if (left < right && left_passed)
        {
            odometry->set_y(left);
        }
        else if (right_passed)
        {
            odometry->set_y(144 - right);
        }
        
        if (front_passed)
        {
            odometry->set_x(front);
        }
    }
    else
    {
        WARN_TEXT("Heading not within range for reset!");
    }
    
    lib15442c::Vec end_pos = odometry->get_position();

    if (abs(initial_pos.x - end_pos.x) > 7)
    {
        WARN_TEXT("Distance reset >7 inches off on x from odom position, ignoring x reset!");
        end_pos.x = initial_pos.x;
    }

    if (abs(initial_pos.y - end_pos.y) > 7)
    {
        WARN_TEXT("Distance reset >7 inches off on y from odom position, ignoring y reset!");
        end_pos.y = initial_pos.y;
    }

    printf("(%f, %f) changed to (%f, %f)\n", initial_pos.x, initial_pos.y, end_pos.x, end_pos.y);
}