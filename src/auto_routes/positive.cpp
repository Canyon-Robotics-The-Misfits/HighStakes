#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::positive)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(72 + 24 + 12, 15.5)); // TODO: update start position

    // Rush neutral mobile goal
    clamp.retract();
    // drive_controller->drive(-5, { min_speed: 110, chained: true });
    drive_controller->boomerang(pose(144 - 48, 48, 315_deg), { backwards: true, lead: 0.4, threshold: 7, max_speed: 100 });
    drive_controller->drive_time(-25, 70);
    clamp.extend();
    pros::delay(100);

    // Intake ring
    drive_controller->drive_time(127, 200);
    intake->move(127);
    drive_controller->faceAngle(90_deg, { threshold: 5_deg });
    drive_controller->drive_time(80, 600);
    drivetrain->move(0, 0);
    pros::delay(800);

    // Drop Goal
    drive_controller->faceAngle(270_deg);

    // Intake last ring
    drive_controller->boomerang(pos(72, 24));

    // Touch ladder
    arm->set_target(mechanism::ArmTarget::NEUTRAL_STAKE);
    drive_controller->boomerang(pos(72 + 36, 36), { backwards: true, threshold: 5 });
    drive_controller->faceAngle(315_deg);
    arm->await_target(5);
    drive_controller->drive_time(127, 500);
}