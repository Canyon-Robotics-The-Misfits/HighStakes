#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(144 - 35, 10.5 + 10));
    
    // pickup goal
    clamp.retract();
    // drive_controller->boomerang(pose(144 - 48, 48, -30_deg), { backwards: true, lead: 0.5, threshold: 6, angle_priority_threshold: 10, max_speed: 110 });
    drive_controller->drive_to(pose(144 - 48, 48, -33_deg), { backwards: true, threshold: 6, max_speed: 110 });
    clamp.extend();
    drive_controller->drive_time(-100, 100);
    pros::delay(50);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(100);
    drive_controller->drive_time(100, 200);
    
    // get line rings
    drive_controller->drive_to(pose(144 - 26, 72 - 8, 90_deg), { max_speed: 110 });
    pros::delay(250);
    drive_controller->drive_time(127, 350);
    pros::delay(200);
    drivetrain->move(-100, -100);
    pros::delay(150);
    drivetrain->move(0, 0);

    // mess up other rings
    drive_controller->facePoint(pos(144 - 24, 48).vec(), 0_deg, { arc_radius: 7 });
    pros::delay(250);

    // get stack
    ring_mech->set_state(mechanism::DISABLED);
    drive_controller->boomerang(pos(144 - 24, 48), { threshold: 5, min_speed: 100 });
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->drive_time(100, 100);
    drive_controller->drive_time(-100, 250);

    // get center ring
    drive_controller->facePoint(pos(144 - 72, 24).vec(), 0_deg, { threshold: 5_deg });
    // drive_controller->drive_to(pose(144 - 48 - 12 + 2.5, 24 + 12 - 7.5, -125_deg));
    drive_controller->drive_to(pose(144 - 48 - 12 + 3, 24 + 12 - 9, -125_deg));
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    oinker.extend();
    pros::delay(200);
    drive_controller->drive_time(-100, 300);
    pros::delay(50);
    oinker.retract();
    pros::delay(250);
    drive_controller->turn(30_deg, { threshold: 10_deg });
    drive_controller->drive_time(127, 300);
    drive_controller->drive_time(-60, 300);
    pros::delay(300);

    // score on alliance stake
    ring_mech->set_state(mechanism::ARM_ALLIANCE_STAKE);
    drive_controller->facePoint(pos(72, 0).vec(), -5_deg, { threshold: 5_deg, min_speed: 22 });
    alliance_stake_adjust.extend();

    double distance = odometry->getPose().vec().distance_to(pos(72, 0).vec());
    drive_controller->drive(distance - 14, { min_speed: 120, chained: true });

    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->drive_time(127, 200);
    ring_mech->set_state(mechanism::DISABLED);
    drive_controller->drive_time(127, 100);
    drive_controller->drive_time(-127, 200);
    alliance_stake_adjust.retract();

    // // clear corner
    // drive_controller->boomerang(pose(144 - 24, 12, 95_deg), { lead: 0.75 });
    // ring_mech->set_state(mechanism::INTAKE_HOOD);
    // drive_controller->faceAngle(98_deg, { threshold: 5_deg });
    // oinker.extend();
    // pros::delay(200);
    // drive_controller->drive_time(60, 300);
    // pros::delay(100);
    // drivetrain->move(0, -100);
    // pros::delay(300);
    // drivetrain->move(0, 100);
    // oinker.retract();
    // pros::delay(200);
    // drive_controller->drive_time(100, 400);
    // drive_controller->drive_time(-100, 250);

    // touch ladder
    drive_controller->boomerang(pos(144 - 40, 40), { backwards: true, threshold: 15, max_speed: 100 });
    pros::delay(100);
    drive_controller->boomerang(pose(144 - (48 + 12 - 4), 48 + 12 - 4, -45_deg), { backwards: true, lead: 0.8, threshold: 2, max_speed: 50 });
    drive_controller->faceAngle(-45_deg + 180_deg);
}