#include "main.h"
#include "autonomous.h"


AUTO_ROUTE(auto_routes::skills)
{
    odometry->initialize(53.5, 13 + 4, -30_deg + 180_deg);
    
    // pickup mogo
    drive_controller->drive_time(-60, 50);
    clamp.extend();
    drive_controller->drive_time(-80, 100);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    pros::delay(700);

    // intake rings
    drive_controller->boomerang(pos(48, 48));
    pros::delay(200);
    drive_controller->boomerang(pos(24, 48));
    pros::delay(200);
    drive_controller->face_point(pos(27, 24).vec(), 0_deg, { threshold: 5_deg });
    drive_controller->boomerang(pose(27, 24, 180_deg), { lead: 0.8, min_speed: 60 });
    pros::delay(200);
    drive_controller->drive_time(80, 450);
    pros::delay(200);
    drivetrain->move(-100, 60);
    pros::delay(700);
    drivetrain->move(0, 0);
    drive_controller->face_point(pos(12, 24).vec(), -10_deg, { threshold: 5_deg });
    drive_controller->drive_time(100, 400);
    pros::delay(400);
    
    // dropoff mogo
    drive_controller->face_point(pos(0, 0).vec(), 180_deg - 10_deg, { threshold: 10_deg });
    pros::delay(400);
    clamp.retract();
    drive_controller->drive_time(-127, 600);
    pros::delay(200);
    drive_controller->drive_time(127, 300);

    // pickup next mogo
    drive_controller->drive_to(pose(144-48 - 20, 24, 90_deg), { r: 25, backwards: true, max_speed: 80 });
    drive_controller->face_point(pos(144 - 48, 24).vec(), 180_deg, { threshold: 5_deg });
    drive_controller->drive_time(-60, 700);
    clamp.extend();
    drive_controller->drive_time(-60, 100);
    pros::delay(150);

    // intake rings
    drive_controller->boomerang(pos(144-48, 48));
    pros::delay(200);
    drive_controller->boomerang(pos(144-24, 48));
    pros::delay(200);
    drive_controller->boomerang(pos(144-10, 72), { threshold: 2 });
    pros::delay(200);
    drive_controller->drive_time(-100, 250);
    drive_controller->face_point(pos(144-24, 24).vec(), 0_deg, { threshold: 5_deg });
    drive_controller->boomerang(pose(144-20, 24, 180_deg), { lead: 0.8 });
    pros::delay(200);
    drive_controller->drive_time(80, 450);
    pros::delay(200);
    drivetrain->move(-100, -40);
    pros::delay(500);
    drivetrain->move(0, 0);
    drive_controller->face_point(pos(144-12, 24).vec(), -5_deg, { threshold: 5_deg });
    drive_controller->drive_time(100, 500);
    pros::delay(600);
    
    // dropoff mogo
    drive_controller->face_point(pos(144, 0).vec(), 180_deg, { threshold: 10_deg });
    pros::delay(400);
    clamp.retract();
    drive_controller->drive_time(-127, 600);
    pros::delay(200);
    drive_controller->drive_time(127, 300);

    // grab ring
    drive_controller->boomerang(pose(144 - 48, 72 + 24, -35_deg), { lead: 0.4, threshold: 3 });
    ring_mech->set_state(mechanism::DISABLED);

    // push mogo
    drive_controller->boomerang(pose(144 - 48 - 4, 144 - 24, 20_deg), { lead: 0.4 });
    drive_controller->boomerang(pose(144 - 30, 144 - 16, 90_deg), { lead: 0.8, min_speed: 90 });
    drivetrain->move(-100, -40);
    pros::delay(500);
    drivetrain->move(0, 0);
    drive_controller->drive_time(-127, 700);
    drive_controller->face_angle(-90_deg, { threshold: 7_deg });

    // push other goal in corner
    drive_controller->boomerang(pose(35, 144 - 15, -90_deg), { lead: 0.6 });
    drivetrain->move(127, 60);
    pros::delay(500);
    drivetrain->move(0, 0);
    drive_controller->drive_time(-80, 300);

    // grab next goal
    drive_controller->drive_to(pose(72 - 20, 144 - 24, 90_deg), { r: 14, backwards: true, max_speed: 80 });
    drive_controller->face_point(pos(144 - 48, 24).vec(), 180_deg, { threshold: 5_deg });
    drive_controller->drive_time(-60, 700);
    clamp.extend();
    drive_controller->drive_time(-60, 100);
    pros::delay(150);

    // pickup rings
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->boomerang(pos(144 - 24, 144 - 48));
    pros::delay(600);
    drive_controller->face_angle(-90_deg);
    drive_controller->boomerang(pose(72, 72 - 4, -135_deg), { lead: 0.4 });
    drive_controller->boomerang(pose(48, 144 - 48, -45_deg), { lead: 0.4 });
    drive_controller->boomerang(pos(24, 144 - 48 - 4));
    drive_controller->boomerang(pos(15, 72 - 6));
    pros::delay(400);

    // pickup rings for wall stake
    drive_controller->face_angle(0_deg);
    ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    drive_controller->boomerang(pos(15, 144 - 24));
    drive_controller->drive_time(-100, 400);
    drive_controller->boomerang(pos(24, 144 - 24));
    drive_controller->drive_time(-100, 400);
    
    // score wall stake
    drive_controller->boomerang(pos(24, 72), { backwards: true, max_speed: 80 });
    drive_controller->face_point(pos(2, 72).vec(), 0_deg, { min_speed: 23 });
    ring_mech->set_state(mechanism::ARM_NEUTRAL_STAKE);
    pros::delay(500);
    drive_controller->boomerang(pos(2, 72), { threshold: 12 });
    ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    pros::delay(300);
    drive_controller->drive_time(-100, 300);
    alliance_stake_adjust.extend();

    // get more rings for wall stake
    drive_controller->boomerang(pose(72, 144 - 24, 90_deg), { lead: 0.4, threshold: 10, min_speed: 100 });
    alliance_stake_adjust.retract();
    drive_controller->boomerang(pos(144 - 24, 144 - 24));
    drive_controller->boomerang(pos(144 - 48, 144 - 48), { backwards: true, threshold: 5 });
    drive_controller->boomerang(pos(144 - 12, 144 - 24));
    
    // score wall stake
    drive_controller->boomerang(pos(144 - 24, 72), { backwards: true, max_speed: 80 });
    drive_controller->face_point(pos(144 - 2, 72).vec(), 0_deg, { min_speed: 23 });
    ring_mech->set_state(mechanism::ARM_NEUTRAL_STAKE);
    pros::delay(500);
    drive_controller->boomerang(pos(144 - 2, 72), { threshold: 12 });
    ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    pros::delay(300);
    drive_controller->drive_time(-100, 300);

    // climb
    drive_controller->boomerang(pos(144 - 48, 144 - 48), { backwards: true });
    drive_controller->face_angle(-135_deg);
    ring_mech->set_state(mechanism::ARM_NEUTRAL_STAKE);
    pros::delay(600);
    drive_controller->drive_time(80, 500);
    ring_mech->set_state(mechanism::DISABLED);
    drive_controller->drive_time(-127, 1000);
}