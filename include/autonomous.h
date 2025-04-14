#pragma once

#include "main.h"
#include "mechanism/ring_manager.hpp"
#include "mechanism/arm.hpp"
#include "gui/gui.h"

#define AUTO_ROUTE(name) void name(                                 \
    std::shared_ptr<lib15442c::DriveController> drive_controller,   \
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,             \
    std::shared_ptr<lib15442c::TrackerOdom> odometry,               \
    std::shared_ptr<mechanism::RingManager> intake,                 \
    std::shared_ptr<mechanism::Arm> arm,                            \
    lib15442c::Pneumatic clamp,                                     \
    lib15442c::Pneumatic doinker,                                   \
    lib15442c::Pneumatic intake_lift,                               \
    gui::AllianceColor alliance                                     \
)


#define AUTO_ROUTE_PARAM(name, param) void name(                    \
    std::shared_ptr<lib15442c::DriveController> drive_controller,   \
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,             \
    std::shared_ptr<lib15442c::TrackerOdom> odometry,               \
    std::shared_ptr<mechanism::RingManager> intake,                 \
    std::shared_ptr<mechanism::Arm> arm,                            \
    lib15442c::Pneumatic clamp,                                     \
    lib15442c::Pneumatic doinker,                                   \
    lib15442c::Pneumatic intake_lift,                               \
    gui::AllianceColor alliance,                                    \
    param                                                           \
)

#define RUN_AUTO(auto_route) auto_route(drive_controller, drivetrain, odometry, intake, arm, clamp, doinker, intake_lift, alliance)
#define RUN_AUTO_PARAM(auto_route, param) auto_route(drive_controller, drivetrain, odometry, intake, arm, clamp, doinker, intake_lift, alliance, param)
#define WAIT_UNTIL(condition) while (!(condition)) { pros::delay(20); }

namespace auto_routes
{
    // AUTO_ROUTE(right_safe);
    // AUTO_ROUTE(left_safe);

    // AUTO_ROUTE_PARAM(positive_red, bool elims);
    // AUTO_ROUTE_PARAM(positive_blue, bool elims);
    // AUTO_ROUTE_PARAM(negative_red, bool elims);
    // AUTO_ROUTE_PARAM(negative_blue, bool elims);

    // AUTO_ROUTE(solo_red);
    // AUTO_ROUTE(solo_blue);

    // AUTO_ROUTE(skills);

    // // route segments
    // AUTO_ROUTE(red_rush_segment);
    // AUTO_ROUTE(blue_rush_segment);
    // AUTO_ROUTE(skills_start_segment);

    // // tests
    // AUTO_ROUTE(skills_triple_test);
    // AUTO_ROUTE(corner_clear);
    // AUTO_ROUTE(mp_test);
};