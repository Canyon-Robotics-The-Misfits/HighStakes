#include "main.h"
#include "autonomous.h"

// void drive_to(lib15442c::Vec point, double offset, std::shared_ptr<lib15442c::DriveController> drive_controller, std::shared_ptr<lib15442c::IOdometry> odometry, lib15442c::Angle angle_offset = 0_deg)
// {
//     drive_controller->facePoint(point, angle_offset, { threshold: 5_deg });

//     double distance = odometry->getPose().vec().distance_to(point);
//     drive_controller->drive(distance);
// }

AUTO_ROUTE(auto_routes::skills)
{
    // drive_controller->boomerang(pose(-24, -24, 90_deg), { backwards: true, lead: 0.5, max_speed: 50 });

    // drive_controller->facePoint(lib15442c::Vec(24, 24));
    // drive_controller->facePoint(lib15442c::Vec(24, -24));
    // drive_controller->facePoint(lib15442c::Vec(-24, -24));
    // drive_controller->facePoint(lib15442c::Vec(-24, 24));

    drive_controller->drive_to(pose(0, 24, 0_deg));
    // drive_controller->faceAngle(180_deg);
    // drive_controller->faceAngle(270_deg);
    // drive_controller->faceAngle(0_deg);

    pros::delay(15000);
}