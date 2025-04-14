#pragma once

#include "config.h"
#include "math.h"

#include "lib15442c/chasis/odometry.hpp"
#include "pros/distance.hpp"

namespace mechanism
{
    using namespace lib15442c::literals;

    constexpr double RESET_THRESHOLD_DEG = 5; 

    constexpr double MM_TO_IN = 0.0393701;
    constexpr double LEFT_DISTANCE_OFFSET = 0;
    constexpr double RIGHT_DISTANCE_OFFSET = 0;
    constexpr double FRONT_DISTANCE_OFFSET = 0;

    void distance_reset(std::shared_ptr<lib15442c::TrackerOdom> odometry)
    {
        pros::Distance left_sensor = pros::Distance(config::PORT_DISTANCE_LEFT);
        pros::Distance right_sensor = pros::Distance(config::PORT_DISTANCE_RIGHT);
        pros::Distance front_sensor = pros::Distance(config::PORT_DISTANCE_FRONT);

        double left = left_sensor.get() * MM_TO_IN + LEFT_DISTANCE_OFFSET;
        double right = right_sensor.get() * MM_TO_IN + RIGHT_DISTANCE_OFFSET;
        double front = front_sensor.get() * MM_TO_IN + FRONT_DISTANCE_OFFSET;

        lib15442c::Angle heading = odometry->get_rotation();

        if (abs(heading.deg()) < RESET_THRESHOLD_DEG)
        {
            odometry->set_y(144 - front);

            if (left < right)
            {
                odometry->set_x(left);
            }
            else
            {
                odometry->set_x(144 - right);
            }
        }
        else if (abs((heading - 90_deg).deg()) < RESET_THRESHOLD_DEG)
        {
            if (left < right)
            {
                odometry->set_y(144 - left);
            }
            else
            {
                odometry->set_y(right);
            }
            
            odometry->set_x(144 - front);
        }
        else if (abs((heading - 180_deg).deg()) < RESET_THRESHOLD_DEG)
        {
            odometry->set_y(front);

            if (left < right)
            {
                odometry->set_x(left);
            }
            else
            {
                odometry->set_x(144 - right);
            }
        }
        else if (abs((heading - 270_deg).deg()) < RESET_THRESHOLD_DEG)
        {
            if (left < right)
            {
                odometry->set_y(left);
            }
            else
            {
                odometry->set_y(144 - right);
            }
            
            odometry->set_x(front);
        }
    }
} // namespace mechanism
