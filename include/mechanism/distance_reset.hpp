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

    void distance_reset(std::shared_ptr<lib15442c::TrackerOdom> odometry);
} // namespace mechanism
