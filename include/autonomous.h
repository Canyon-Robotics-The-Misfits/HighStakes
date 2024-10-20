#pragma once

#include "main.h"
#include "mechanism/ring_mech.h"
#include "gui/gui.h"

#define AUTO_ROUTE(name) void name(                                 \
    std::shared_ptr<lib15442c::DriveController> drive_controller,   \
    std::shared_ptr<lib15442c::IDrivetrain> drivetrain,             \
    std::shared_ptr<lib15442c::IOdometry> odometry,                 \
    std::shared_ptr<mechanism::RingMech> ring_mech,                 \
    lib15442c::Pneumatic clamp,                                     \
    lib15442c::Pneumatic oinker,                                    \
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