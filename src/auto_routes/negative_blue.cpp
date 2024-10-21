#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(144 - 35, 10.5 + 10));
    
    // pickup goal
    clamp.retract();
    drive_controller->boomerang(pose(144 - 48, 48, -25_deg), { backwards: true, lead: 0.85, threshold: 8, angle_priority_threshold: 10, max_speed: 100 });
    drive_controller->drive_time(-60, 75);
    clamp.extend();
    drive_controller->drive_time(-60, 100);
    pros::delay(100);
    
    // get line rings
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->boomerang(pose(144 - 24, 72 - 7, 90_deg), { lead: 0.6, max_speed: 110 });
    pros::delay(250);
    drive_controller->drive_time(127, 250);

    // mess up other rings
    drive_controller->facePoint(pos(144 - 24, 48).vec(), 0_deg, { threshold: 3_deg });
    ring_mech->set_state(mechanism::INTAKE_OUTTAKE); // agitate rings if jammed
    pros::delay(50);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(200);

    // get stack
    drive_controller->boomerang(pos(144 - 24, 48), { min_speed: 80 });
    ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    drive_controller->drive_time(80, 100);
    drive_controller->drive_time(-80, 250);

    // get center ring
    drive_controller->facePoint(pos(144 - 72, 24).vec());
    drive_controller->boomerang(pose(144 - 48 - 12 + 2, 24 + 12 - 9, -125_deg), { max_speed: 100 });
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    oinker.extend();
    pros::delay(100);
    drive_controller->drive_time(-80, 400);
    pros::delay(50);
    oinker.retract();
    drive_controller->turn(25_deg);
    drive_controller->drive_time(100, 300);
    drive_controller->drive_time(-100, 300);

    // score on alliance stake
    ring_mech->set_state(mechanism::ARM_ALLIANCE_STAKE);
    drive_controller->facePoint(pos(72, 0).vec());
    pros::delay(200);
    alliance_stake_adjust.extend();
    drive_controller->drive_time(80, 500);
    pros::delay(100);
    drive_controller->drive_time(-100, 250);
}