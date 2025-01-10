#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_red)
{
    odometry->initialize(58 -8, 16 +5+8-17, 143.5_deg); // start position is way off the actual one but it works for some reason

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    pros::delay(250);
    drive_controller->drive(9, { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    drivetrain->move(0, 40);
    pros::delay(200);

    // grab goal
    drive_controller->boomerang(pos(48-10, 48-10), { backwards: true, threshold: 3, min_speed: 60 });
    drive_controller->drive_time(-60, 50);
    clamp.extend();
    intake->set_state(IntakeState::HOOD);
    
    // get line rings
    drive_controller->drive_to(pose(29, 72 - 12 + 6-4, -90_deg), { max_speed: 110 });
    pros::delay(200);
    drive_controller->drive_time(127, 300);
    pros::delay(200);
    intake->set_state(IntakeState::DISABLED);

    // get stack
    drive_controller->boomerang(pos(48, 48), { backwards: true, threshold: 2, min_speed: 40 });
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(24, 48), { threshold: 3, min_speed: 80 });
    drive_controller->drive_time(80, 300);
    pros::delay(100);
    drive_controller->drive(-10, { min_speed: 60, chained: true });

    // get center ring
    drive_controller->boomerang(pos(48-10, 24-8));
    drive_controller->face_angle(90_deg, { chained: true });
    drive_controller->face_angle(90_deg, { max_speed: 40 });
    arm->set_state(ArmState::LADDER_TOUCH);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->drive(26);
    pros::delay(300);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(14, { max_speed: 60, min_speed: 40, chained: true });
    pros::delay(200);

    // touch ladder
    drive_controller->boomerang(pos(48 -24, 48 - 24), { backwards: true, threshold: 2, min_speed: 60 } );
    drive_controller->face_angle(45_deg, { threshold: 8_deg });
    drive_controller->drive_time(100, 500);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
}