#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::negative)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(48, 24)); // TODO: update start position

    // Grab mogo
    drive_controller->drive(20);
    clamp.extend();
    pros::delay(100);

    // intake rings
    intake->move(127);
    drive_controller->drive_to(pose(24, 48, 350_deg), { threshold: 10, min_speed: 60 });
    drive_controller->drive_to(pose(24 - 4, 72 - 4, 10_deg), { threshold: 10 });
    drive_controller->drive_time(-127, 250);
    drive_controller->drive_to(pose(24 + 4, 72 - 4, 15_deg), { threshold: 10 });

    // pickup ring for wall stake
    auto drive_to_pickup = drive_controller->drive_to(pose(72, 24, 110_deg), { threshold: 4, async: true });
    pros::delay(500);
    intake->set_redirect_mode(mechanism::IntakeRedirectMode::ALL);
    drive_to_pickup->await();

    // score on wall stake
    arm->set_target(mechanism::ArmTarget::ALLIANCE_STAKE);
    drive_controller->faceAngle(180_deg);
    arm->await_target(5);
    drive_controller->drive(6);
    arm->set_target(mechanism::ArmTarget::LOAD);
    pros::delay(500);
    drive_controller->drive_time(-127, 250);

    // touch ladder
    arm->set_target(mechanism::ArmTarget::NEUTRAL_STAKE);
    drive_controller->boomerang(pos(32, 32), { backwards: true, threshold: 5 });
    drive_controller->faceAngle(45_deg);
    arm->await_target();
    drive_controller->drive_time(127, 500);
}