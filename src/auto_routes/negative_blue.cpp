#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(144 - 35, 10.5 + 10));
    
    // pickup goal
    clamp.retract();
    // drive_controller->boomerang(pose(144 - 48, 48, -30_deg), { backwards: true, lead: 0.5, threshold: 6, angle_priority_threshold: 10, max_speed: 110 });
    drive_controller->drive_to(pose(144 - 48, 48, -30_deg), { backwards: true, threshold: 6, max_speed: 110 });
    clamp.extend();
    drive_controller->drive_time(-100, 100);
    pros::delay(50);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(100);
    drive_controller->drive_time(100, 200);
    
    // get line rings
    drive_controller->drive_to(pose(144 - 24, 72 - 7, 90_deg), { max_speed: 110 });
    pros::delay(250);
    drive_controller->drive_time(127, 300);
    pros::delay(200);

    // mess up other rings
    drive_controller->facePoint(pos(144 - 24, 48).vec(), 0_deg, { threshold: 3_deg });
    pros::delay(150);

    // get stack
    drive_controller->boomerang(pos(144 - 24, 48), { min_speed: 100 });
    ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    drive_controller->drive_time(80, 100);
    drive_controller->drive_time(-80, 350);

    // get center ring
    drive_controller->facePoint(pos(144 - 72, 24).vec(), 0_deg, { threshold: 5_deg });
    // drive_controller->drive_to(pose(144 - 48 - 12 + 2.5, 24 + 12 - 7.5, -125_deg));
    drive_controller->drive_to(pose(144 - 48 - 12 + 3, 24 + 12 - 7, -125_deg));
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    oinker.extend();
    pros::delay(200);
    drive_controller->drive_time(-80, 400);
    pros::delay(50);
    oinker.retract();
    drive_controller->turn(25_deg);
    drive_controller->drive_time(100, 300);
    drive_controller->drive_time(-100, 300);
    return;

    // score on alliance stake
    ring_mech->set_state(mechanism::ARM_ALLIANCE_STAKE);
    drive_controller->facePoint(pos(72, 0).vec());
    pros::delay(200);
    alliance_stake_adjust.extend();
    drive_controller->drive_time(80, 500);
    pros::delay(100);
    drive_controller->drive_time(-100, 250);
    

    // clear corner
    // ring_mech->set_state(mechanism::INTAKE_HOOD);
    // drive_controller->boomerang(pose(144 - 24, 12, 95_deg), { lead: 0.9 });
    // oinker.extend();
    // drive_controller->drive_time(60, 300);
    // drivetrain->move(0, -100);
    // pros::delay(200);
    // drivetrain->move(0, 100);
    // pros::delay(200);
    // drive_controller->drive_time(80, 150);
    // drive_controller->drive_time(-100, 250);
}