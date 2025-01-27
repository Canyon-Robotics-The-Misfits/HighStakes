#pragma once

#include "main.h"
#include "mechanism/intake.h"
#include "mechanism/arm.h"
#include "gui/gui.h"

#define AUTO_ROUTE(name) void name(                                 \
    std::shared_ptr<lib15442c::DriveController> drive_controller,   \
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,             \
    std::shared_ptr<lib15442c::TrackerOdom> odometry,               \
    std::shared_ptr<mechanism::Intake> intake,                      \
    std::shared_ptr<mechanism::Arm> arm,                            \
    lib15442c::Pneumatic clamp,                                     \
    lib15442c::Pneumatic doinker,                                   \
    gui::AllianceColor alliance                                     \
)

#define RUN_AUTO(auto_route) auto_route(drive_controller, drivetrain, odometry, intake, arm, clamp, doinker, alliance)

namespace auto_routes
{
    using mechanism::IntakeState;
    using mechanism::ArmState;

    AUTO_ROUTE(right_safe);
    AUTO_ROUTE(left_safe);

    AUTO_ROUTE(positive_red);
    AUTO_ROUTE(positive_blue);
    AUTO_ROUTE(negative_red);
    AUTO_ROUTE(negative_blue);

    AUTO_ROUTE(solo_red);
    AUTO_ROUTE(solo_blue);

    AUTO_ROUTE(skills);

    // route segments
    AUTO_ROUTE(red_rush_segment);
    AUTO_ROUTE(blue_rush_segment);
    AUTO_ROUTE(skills_start_segment);
};