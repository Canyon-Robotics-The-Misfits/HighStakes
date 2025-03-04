#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->initialize(144-57, 13, 301_deg);

    // get center ring
    arm->set_state(ArmState::ALLIANCE_STAKE);
    pros::delay(100);
    intake_lift.extend();
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(13, { min_speed: 35, chained: true });
    intake_lift.retract();
    pros::delay(100);
    auto backup_center_ring = drive_controller->boomerang(pos(72+9.5 + 5, 10.5 + 5), { backwards: true, min_speed: 50, chained: true, async: true });
    intake->set_state(IntakeState::DISABLED);
    // pros::delay(150);
    backup_center_ring->await();

    // get alliance stake
    // drive_controller->face_point(lib15442c::Vec(72+9.5, 10.5), 5_deg, { threshold: 3_deg, min_speed: 30, chained: true });
    drive_controller->drive_to(pose(72+9.5, 10.5, -145_deg), { r: 0.5, timeout: 2000, min_speed: 50 });
    arm->set_state(ArmState::LOAD);
    pros::delay(150);

    // grab goal
    // drive_controller->boomerang(pose(48 -2, 48, -15_deg + 180_deg), { backwards: true, lead: 0.7, threshold: 3, angle_priority_threshold: 12, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 5, min_speed: 60 });
    // drive_controller->drive_time(-60, 100);
    clamp.extend();
    pros::delay(150);
    intake->set_state(IntakeState::HOOD);

    // get stack
    drive_controller->boomerang(pos(144 - 24, 48), { threshold: 6, min_speed: 80, chained: true });
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 6, min_speed: 80, chained: true });
    
    // get line rings
    auto drive_to_line = drive_controller->drive_to(pose(144 - 21 -11, 72 - 12 + 3, 75_deg), { r: 7, max_speed: 110, min_speed: 40, async: true });
    pros::delay(500);
    intake->set_state(IntakeState::HOOD);
    drive_to_line->await();
    drive_controller->face_angle(95_deg, { chained: true });
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->drive(3, { min_speed: 60, chained: true });
    pros::delay(100);

    // score on wall stake
    auto drive_to_wall_stake = drive_controller->drive_to(pose(144 - 13, 72 - 6.5, 50_deg), { r: 4, timeout: 1500, min_speed: 30, async: true });
    pros::delay(900);
    intake->set_state(IntakeState::DISABLED);
    intake_lift.extend();
    drive_to_wall_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-12, { min_speed: 80, chained: true });
    intake_lift.retract();

    // get corner 
    intake->set_state(IntakeState::HOOD);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->face_point(lib15442c::Vec(144 - 16, 16), -10_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 16 +1, 16 +1), { threshold: 2 });
    drive_controller->face_point(lib15442c::Vec(144, 0), -10_deg);
    pros::delay(50);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_time(60, 400);
    pros::delay(150);
    drive_controller->drive(-4, { min_speed: 40, chained: true });
    pros::delay(150);
    intake_lift.extend();
    pros::delay(100);
    drive_controller->drive_time(60, 350);
    pros::delay(250);
    intake_lift.retract();
    drive_controller->drive(-10, { min_speed: 60, chained: true });

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    // drive_controller->face_angle(-45_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 36, 36), { backwards: true, threshold: 2, timeout: 3000, min_speed: 127 } );
    intake->set_state(IntakeState::REVERSE);
    drive_controller->face_angle(-40_deg, { threshold: 10_deg, chained: true  });
    arm->set_state(ArmState::DISABLED);
    // drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
}