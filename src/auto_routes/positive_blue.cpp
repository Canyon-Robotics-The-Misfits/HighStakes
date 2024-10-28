#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::positive_blue)
{
    odometry->setRotation(0_deg);
    odometry->setPosition(lib15442c::Vec(35, 18.5 + 10 - 5));

    // rush goal
    drive_controller->drive_time(127, 300);
    drive_controller->drive_to(pose(24 + 10, 48 + 9, 0_deg), { r: 10, threshold: 3, timeout: 4000 });
    drive_controller->faceAngle(-55_deg, { threshold: 10_deg, min_speed: 30, chained: true });
    oinker.extend();
    pros::delay(75);
    drive_controller->faceAngle(-90_deg, { threshold: 10_deg, min_speed: 30, chained: true });
    oinker.retract();
    // pros::delay(100);
    drive_controller->drive_time(100, 100);
    drivetrain->move(0, -100);
    pros::delay(100);
    drivetrain->move(0, 0);

    // pickup next goal
    drive_controller->boomerang(pose(48, 48, -30_deg + 180_deg), { backwards: true, lead: 0.4, threshold: 6 });
    drive_controller->drive_time(-100, 150);
    clamp.extend();
    drive_controller->drive_time(-100, 100);
    pros::delay(50);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(100);

    // intake
    drive_controller->boomerang(pos(24, 48));
    drive_controller->drive_time(-100, 250);
    
    // (dont) get the center ring
    drive_controller->facePoint(pos(48 + 12 - 6, 24 + 12 - 9).vec(), 25_deg, { threshold: 10_deg });
    drive_controller->boomerang(pos(48 + 12 - 6, 24 + 12 - 9));

    // score on alliance stake
    ring_mech->set_state(mechanism::ARM_ALLIANCE_STAKE);
    drive_controller->facePoint(pos(72, 3).vec(), 2_deg, { threshold: 1_deg, min_speed: 23 });
    alliance_stake_adjust.extend();
    pros::delay(100);

    drive_controller->boomerang(pos(72, 2), { threshold: 14 });

    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->drive_time(127, 100);
    ring_mech->set_state(mechanism::DISABLED);
    drive_controller->drive_time(127, 50);
    drive_controller->drive_time(-127, 300);
    alliance_stake_adjust.retract();

    // clear corner
    drive_controller->boomerang(pose(14, 26, 192_deg), { lead: 0.45, threshold: 3, timeout: 3500 });
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->faceAngle(192_deg, { threshold: 5_deg, max_speed: 40 });
    oinker.extend();
    pros::delay(200);
    drive_controller->drive_time(100, 400);
    pros::delay(100);
    drivetrain->move(0, -127);
    pros::delay(400);
    drivetrain->move(0, 127);
    oinker.retract();
    pros::delay(400);
    drive_controller->drive_time(100, 400);
    drive_controller->faceAngle(-135_deg, { threshold: 5_deg });
    drive_controller->drive_time(-100, 200);

    // dropoff goal
    drive_controller->faceAngle(135_deg, { threshold: 5_deg });
    clamp.retract();
    pros::delay(200);
    drive_controller->drive_time(100, 100);
    drive_controller->faceAngle(0_deg, { threshold: 5_deg });
    drive_controller->drive_time(127, 150);
    drive_controller->faceAngle(180_deg, { threshold: 5_deg });

    // // touch ladder
    // drive_controller->boomerang(pos(40, 35), { backwards: true, threshold: 10, min_speed: 80 });
    // drive_controller->boomerang(pos(48 + 12 - 4.5 + 3, 48 + 12 - 4.5 - 3), { backwards: true, lead: 0.4, threshold: 1.5, angle_priority_threshold: 0, max_speed: 50 });
    // drive_controller->faceAngle(45_deg + 180_deg);
}