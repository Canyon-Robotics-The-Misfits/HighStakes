#pragma once

#include "main.h"
#include "mechanism/intake.h"
#include "mechanism/arm.h"
#include "gui/gui.h"

#define AUTO_ROUTE(name) void name(                                 \
    std::shared_ptr<lib15442c::DriveController> drive_controller,   \
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,             \
    std::shared_ptr<lib15442c::IOdometry> odometry,                 \
    std::shared_ptr<mechanism::Intake> intake,                      \
    std::shared_ptr<mechanism::Arm> arm,                            \
    lib15442c::Pneumatic clamp,                                     \
    lib15442c::Pneumatic doinker,                                   \
    gui::AllianceColor alliance                                     \
)

namespace auto_routes
{
    AUTO_ROUTE(right_safe);
    AUTO_ROUTE(left_safe);

    AUTO_ROUTE(positive_red);
    AUTO_ROUTE(positive_blue);
    AUTO_ROUTE(negative_red);
    AUTO_ROUTE(negative_blue);

    AUTO_ROUTE(solo);

    AUTO_ROUTE(skills);
};