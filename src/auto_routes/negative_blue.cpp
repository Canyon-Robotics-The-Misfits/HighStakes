#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->initialize(144-40, 21, 13.5_deg);

    // ring rush
    intake->set_state(IntakeState::WALL_STAKE);
    doinker.extend();
    drive_controller->drive(38, { min_speed: 40, chained: true });
    drive_controller->drive(-15, { min_speed: 40, chained: true });
    doinker.retract();
    pros::delay(50);

    // pickup goal
    drive_controller->face_point(lib15442c::Vec(144 - 48, 48), 180_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 5, min_speed: 60, chained: true });
    drive_controller->drive_time(-60, 150);
    clamp.extend();
    pros::delay(100);

    // get stack
    arm->set_state(ArmState::NEUTRAL_STAKE);
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 24, 48 + 4), { threshold: 6, min_speed: 40 });
    pros::delay(200);

    // wall stake
    auto drive_to_wall_stake = drive_controller->drive_to(pose(144 - 11.5, 72 - 8.5, 45_deg), { r: 4, timeout: 1500, min_speed: 30, async: true });
    pros::delay(1100);
    intake->set_state(IntakeState::DISABLED);
    intake_lift.extend();
    drive_to_wall_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    intake_lift.retract();

    // get corner 
    intake->set_state(IntakeState::HOOD);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->face_point(lib15442c::Vec(144 - 24, 20), 0_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 24, 26), { threshold: 4, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 18, 18), { threshold: 2, min_speed: 30 });

    drive_controller->face_point(lib15442c::Vec(144, 0), -10_deg, { threshold: 3_deg });
    pros::delay(50);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_time(60, 500);
    pros::delay(150);
    drive_controller->drive(-3, { min_speed: 40, chained: true });
    pros::delay(150);
    intake_lift.extend();
    pros::delay(100);
    drive_controller->drive_time(60, 250);
    pros::delay(250);
    intake_lift.retract();
    drive_controller->drive(-10, { min_speed: 60, chained: true });
}