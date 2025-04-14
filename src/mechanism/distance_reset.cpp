#include "distance_reset.hpp"

#define LOGGER "distance_reset.cpp"

void mechanism::distance_reset(std::shared_ptr<lib15442c::TrackerOdom> odometry)
{
    pros::Distance left_sensor = pros::Distance(config::PORT_DISTANCE_LEFT);
    pros::Distance right_sensor = pros::Distance(config::PORT_DISTANCE_RIGHT);
    pros::Distance front_sensor = pros::Distance(config::PORT_DISTANCE_FRONT);

    double left = left_sensor.get() * MM_TO_IN + LEFT_DISTANCE_OFFSET;
    double right = right_sensor.get() * MM_TO_IN + RIGHT_DISTANCE_OFFSET;
    double front = front_sensor.get() * MM_TO_IN + FRONT_DISTANCE_OFFSET;

    bool left_in_range = left_sensor.get() != 9999 && left_sensor.is_installed();
    bool right_in_range = left_sensor.get() != 9999 && right_sensor.is_installed();
    bool front_in_range = left_sensor.get() != 9999 && front_sensor.is_installed();

    lib15442c::Angle heading = odometry->get_rotation();

    if (!(left_in_range || right_in_range))
    {
        WARN_TEXT("No horrizontal sensor in range for reset!");
    }
    
    if (!front_in_range)
    {
        WARN_TEXT("Front sensor out of range for reset!");
    }

    if (abs(heading.deg()) < RESET_THRESHOLD_DEG)
    {
        if (front_in_range)
        {
            odometry->set_y(144 - front);
        }

        if (left < right && left_in_range)
        {
            odometry->set_x(left);
        }
        else if (right_in_range)
        {
            odometry->set_x(144 - right);
        }
    }
    else if (abs((heading - 90_deg).deg()) < RESET_THRESHOLD_DEG)
    {
        if (left < right && left_in_range)
        {
            odometry->set_y(144 - left);
        }
        else if (right_in_range)
        {
            odometry->set_y(right);
        }
        
        if (front_in_range)
        {
            odometry->set_x(144 - front);
        }
    }
    else if (abs((heading - 180_deg).deg()) < RESET_THRESHOLD_DEG)
    {
        if (front_in_range)
        {
            odometry->set_y(front);
        }

        if (left < right && left_in_range)
        {
            odometry->set_x(left);
        }
        else if (right_in_range)
        {
            odometry->set_x(144 - right);
        }
    }
    else if (abs((heading - 270_deg).deg()) < RESET_THRESHOLD_DEG)
    {
        if (left < right && left_in_range)
        {
            odometry->set_y(left);
        }
        else if (right_in_range)
        {
            odometry->set_y(144 - right);
        }
        
        if (front_in_range)
        {
            odometry->set_x(front);
        }
    }
    else
    {
        WARN_TEXT("Heading not within range for reset!");
    }
}