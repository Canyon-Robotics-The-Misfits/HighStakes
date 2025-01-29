#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_red)
{
    odometry->initialize(56, 20, 143.5_deg); // start position is way off the actual one but it works for some reason

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    pros::delay(250);
    drive_controller->drive(8, { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    drivetrain->move(0, 40);
    pros::delay(200);

    // grab goal
    drive_controller->boomerang(pos(48, 48), { backwards: true, threshold: 3, min_speed: 60 });
    drive_controller->drive_time(-60, 200);
    clamp.extend();
    intake->set_state(IntakeState::HOOD);
    
    // get line rings
    drive_controller->drive_to(pose(21, 72 - 12 + 4, -90_deg), { max_speed: 110, min_speed: 25 });
    pros::delay(100);
    drive_controller->drive_time(127, 200);
    pros::delay(100);
    intake->set_state(IntakeState::DISABLED);

    // get stack
    drive_controller->boomerang(pos(48, 48), { backwards: true, threshold: 2, min_speed: 40 });
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(24, 48 + 3), { threshold: 3, min_speed: 80 });
    pros::delay(300);
    drive_controller->drive(-10, { min_speed: 60, chained: true });

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->boomerang(pos(16 -2, 16 +2), { threshold: 2 });
    drive_controller->face_point(lib15442c::Vec(0, 0), 5_deg);
    pros::delay(50);
    drive_controller->drive_time(50, 1000);
    drive_controller->drive_time(-100, 200);

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    drive_controller->boomerang(pos(36, 36), { backwards: true, threshold: 2, min_speed: 60 } );
    drive_controller->face_angle(45_deg);
    drive_controller->drive_time(60, 700);
    arm->set_state(ArmState::DISABLED);
    // drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
}