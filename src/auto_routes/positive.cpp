#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::positive)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(72 + 24 + 12, 24)); // TODO: update start position

    // Rush neutral mobile goal
    drive_controller->drive(-24, { min_speed: 110, chained: true });
    drive_controller->drive_to(pose(144 - 24, 72, 180_deg + 45_deg), { backwards: true, threshold: 10 });
    clamp.extend();
    pros::delay(100);

    // Intake ring
    intake->move(127);
    drive_controller->facePoint(lib15442c::Vec(144 - 24, 48), 0_deg, { threshold: 5_deg, chained: true });
    drive_controller->drive_time(127, 500);

    // Drop Goal
    drive_controller->faceAngle(330_deg, { chained: true });
    clamp.retract();

    // Grab second goal
    drive_controller->facePoint(lib15442c::Vec(144 - 48, 48), 180_deg, { threshold: 5_deg });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 10 });
    clamp.extend();
    pros::delay(100);

    // Intake last ring
    drive_controller->boomerang(pos(72, 24));

    // Touch ladder
    arm->set_target(mechanism::ArmTarget::NEUTRAL_STAKE);
    drive_controller->boomerang(pos(72 + 36, 36), { backwards: true, threshold: 5 });
    drive_controller->faceAngle(315_deg);
    arm->await_target(5);
    drive_controller->drive_time(127, 500);
}