#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_red)
{
    odometry->initialize(54, 19, 51.5_deg);

    // get center ring and alliance stake
    doinker.extend();
    arm->set_state(ArmState::ALLIANCE_STAKE);
    pros::delay(350);
    drive_controller->drive(-12, { min_speed: 40, chained: true });
    doinker.retract();
    pros::delay(50);
    drive_controller->face_angle(80_deg, { chained: true });
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(10, { min_speed: 40, chained: true });
    auto face_alliance_stake = drive_controller->face_point(lib15442c::Vec(72, 0), 10_deg, { async: true });
    pros::delay(200);
    intake->set_state(IntakeState::DISABLED);
    face_alliance_stake->await();
    drive_controller->drive(5, { min_speed: 35, chained: true });
    arm->set_state(ArmState::LOAD);
    pros::delay(150);

    // grab goal
    // drive_controller->boomerang(pose(48 -2, 48, -15_deg + 180_deg), { backwards: true, lead: 0.7, threshold: 3, angle_priority_threshold: 12, min_speed: 60 });
    drive_controller->boomerang(pos(48 -2, 48), { backwards: true, threshold: 3, min_speed: 60 });
    drive_controller->drive_time(-60, 100);
    clamp.extend();
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    
    // get line rings
    drive_controller->drive_to(pose(21, 72 - 12 + 4, -90_deg), { max_speed: 110, min_speed: 40 });
    drive_controller->drive_time(60, 200);
    pros::delay(100);

    // get stack
    drive_controller->boomerang(pos(48, 48), { backwards: true, threshold: 2, min_speed: 40 });
    drive_controller->boomerang(pos(24, 48 + 3), { threshold: 3, min_speed: 80 });
    pros::delay(300);
    drive_controller->drive(-10, { min_speed: 60, chained: true });

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->boomerang(pos(16 -2, 16 +2), { threshold: 2 });
    drive_controller->face_point(lib15442c::Vec(0, 0), 10_deg);
    pros::delay(50);
    drive_controller->drive_time(60, 500);
    drive_controller->drive_time(-100, 200);

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    drive_controller->boomerang(pos(36, 36), { backwards: true, threshold: 2, min_speed: 60 } );
    drive_controller->face_angle(40_deg, { threshold: 5_deg });
    drive_controller->drive_time(60, 700);
    arm->set_state(ArmState::DISABLED);
    // drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
}