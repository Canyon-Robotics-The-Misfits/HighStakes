#include "main.h"
#include "autonomous.h"

void drive_to(lib15442c::Vec point, double offset, std::shared_ptr<lib15442c::DriveController> drive_controller, std::shared_ptr<lib15442c::IOdometry> odometry, lib15442c::Angle angle_offset = 0_deg_raw)
{
    drive_controller->facePoint(point, angle_offset, { threshold: 5_deg_raw });

    double distance = odometry->getPose().vec().distance_to(point);
    drive_controller->drive(distance);
}

AUTO_ROUTE(auto_routes::skills)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(35, 15.5));

    
    // Rush neutral mobile goal
    clamp.extend();
    oinker.retract();
    arm->move(80);
    pros::delay(250);
    arm->move(-80);
    drive_controller->drive_time(-25, 70);
    clamp.extend();
    pros::delay(250);
    intake->move(127);

    drive_to(lib15442c::Vec(48, 48), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(24, 48), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(24, 12), 0, drive_controller, odometry);
    drive_controller->drive_time(-100, 500);
    drive_to(lib15442c::Vec(12, 24), 0, drive_controller, odometry);
    drivetrain->move(-100, 20);
    pros::delay(400);
    drive_controller->facePoint(lib15442c::Vec(0, 0), 180_deg_raw);
    drive_controller->drive_time(-127, 600);
    clamp.retract();
    pros::delay(250);
    drive_controller->drive_time(127, 300);
    
    drive_to(lib15442c::Vec(96, 24), 5, drive_controller, odometry, 180_deg_raw);
    drive_controller->drive_time(-25, 70);
    clamp.extend();
    pros::delay(250);
    
    drive_to(lib15442c::Vec(96, 48), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(120, 48), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(132, 72), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(120, 48), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(120, 12), 0, drive_controller, odometry);
    drive_controller->drive_time(-100, 500);
    drive_to(lib15442c::Vec(132, 24), 0, drive_controller, odometry);
    drivetrain->move(-100, -20);
    pros::delay(400);
    drive_controller->facePoint(lib15442c::Vec(72, 0), 180_deg_raw);
    drive_controller->drive_time(-127, 600);
    clamp.retract();
    pros::delay(250);
    drive_controller->drive_time(127, 300);
\
    drive_to(lib15442c::Vec(120, 72), 0, drive_controller, odometry);
    drive_to(lib15442c::Vec(96, 132), 0, drive_controller, odometry, 180_deg_raw);
    clamp.extend();
    pros::delay(250);
    drive_controller->facePoint(lib15442c::Vec(144, 144), 180_deg_raw);
    drive_controller->drive_time(-127, 1000);
    clamp.retract();
    pros::delay(250);
    drive_controller->drive_time(127, 300);
    
    clamp.extend();
    pros::delay(250);
    drive_to(lib15442c::Vec(48, 132), 0, drive_controller, odometry, 180_deg_raw);
    drive_controller->facePoint(lib15442c::Vec(144, 144), 180_deg_raw);
    drive_controller->drive_time(-127, 1000);
    clamp.retract();
    pros::delay(250);
    drive_controller->drive_time(127, 300);
    
    drive_controller->facePoint(lib15442c::Vec(72, 120), 180_deg_raw);
    clamp.extend();
    pros::delay(250);
    drive_to(lib15442c::Vec(48, 96), 0, drive_controller, odometry, 180_deg_raw);
    drive_to(lib15442c::Vec(24, 96), 0, drive_controller, odometry, 180_deg_raw);
    drive_to(lib15442c::Vec(12, 72), 0, drive_controller, odometry, 180_deg_raw);
}