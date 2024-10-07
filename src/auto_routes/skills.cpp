#include "main.h"
#include "autonomous.h"

void drive_to(lib15442c::Vec point, double offset, std::shared_ptr<lib15442c::DriveController> drive_controller, std::shared_ptr<lib15442c::IOdometry> odometry, lib15442c::Angle angle_offset = 0_deg)
{
    drive_controller->facePoint(point, angle_offset, { threshold: 5_deg });

    double distance = odometry->getPose().vec().distance_to(point);
    drive_controller->drive(distance);
}

AUTO_ROUTE(auto_routes::skills)
{
    drive_controller->boomerang(pos(24, 48));
}