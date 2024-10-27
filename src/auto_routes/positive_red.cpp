#include "main.h"
#include "autonomous.h"


AUTO_ROUTE(auto_routes::positive_red)
{
    odometry->setRotation(0_deg);
    odometry->setPosition(lib15442c::Vec(144-35, 18.5 + 10 - 5));

    // rush goal
    drive_controller->drive_time(127, 300);
    drive_controller->drive_to(pose(144 - 24 - 10, 48 + 9, 0_deg), { r: 10, threshold: 3, timeout: 4000 });
    drive_controller->faceAngle(10_deg, { threshold: 5_deg, min_speed: 30, chained: true });
    oinker.extend();
    pros::delay(75);
    drive_controller->faceAngle(90_deg, { threshold: 10_deg, min_speed: 30, chained: true });
    oinker.retract();
    // pros::delay(100);
    drive_controller->drive_time(100, 100);
    drivetrain->move(0, -100);
    pros::delay(100);
    drivetrain->move(0, 0);

    // pickup next goal
    drive_controller->boomerang(pose(144 - 48, 48, 30_deg + 180_deg), { backwards: true, lead: 0.4, threshold: 6 });
    clamp.extend();
    drive_controller->drive_time(-100, 75);
    pros::delay(50);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(100);

    // intake
    drive_controller->boomerang(pos(144 - 24, 48));
    drive_controller->drive_time(-100, 250);
    
    // get the center ring
    drive_controller->facePoint(pos(144 - 72, 24).vec(), 0_deg, { threshold: 10_deg });
    drive_controller->drive_to(pose(144 - 48 - 12 + 3, 24 + 12 - 9, -130_deg));
    oinker.extend();
    pros::delay(150);
    drive_controller->drive_time(-100, 300);
    pros::delay(50);
    oinker.retract();
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(150);
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

    // clear corner
    drive_controller->boomerang(pose(144 - 28, 11, 95_deg), { lead: 0.75 });
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->faceAngle(99_deg, { threshold: 5_deg });
    oinker.extend();
    pros::delay(200);
    drive_controller->drive_time(80, 400);
    pros::delay(100);
    drivetrain->move(0, -127);
    pros::delay(300);
    drivetrain->move(0, 127);
    oinker.retract();
    pros::delay(200);
    drive_controller->drive_time(100, 400);
    drive_controller->drive_time(-100, 250);

    // touch ladder
    drive_controller->boomerang(pos(144 - 36, 36), { backwards: true, threshold: 10, min_speed: 80 });
    drive_controller->boomerang(pose(144 - (48 + 12 - 4.5), 48 + 12 - 4.5, -45_deg), { backwards: true, lead: 0.7, threshold: 2.0, max_speed: 50 });
    drive_controller->faceAngle(-45_deg + 180_deg);
}