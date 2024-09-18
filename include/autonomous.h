#pragma once

#include "main.h"
#include "mechanism/intake.h"

#define AUTO_ROUTE(name) void name(                                 \
    std::shared_ptr<lib15442c::DriveController> drive_controller,   \
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,             \
    std::shared_ptr<lib15442c::IOdometry> odometry,                 \
    std::shared_ptr<mechanism::Intake> intake,                      \
    lib15442c::Motor arm,                                           \
    lib15442c::Pneumatic clamp                                      \
)

namespace auto_routes
{
    enum class Route
    {
        NONE,
        POSITIVE,
        NEGATIVE,
        POSITIVE_ELIMS,
        NEGATIVE_ELIMS,
        SOLO,
        SKILLS,
    };

    AUTO_ROUTE(positive);
    AUTO_ROUTE(negative);
    AUTO_ROUTE(positive_elims);
    AUTO_ROUTE(negative_elims);
    AUTO_ROUTE(solo);
    AUTO_ROUTE(skills);
};