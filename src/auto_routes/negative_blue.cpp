#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->initialize(144-56, 20, -143.5_deg); // start position is way off the actual one but it works for some reason

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    pros::delay(250);
    drive_controller->drive(8, { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    drivetrain->move(0, 40);
    pros::delay(200);

    // grab goal
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 3, min_speed: 60 });
    drive_controller->drive_time(-60, 50);
    clamp.extend();
    intake->set_state(IntakeState::HOOD);
    
    // get line rings
    drive_controller->drive_to(pose(144 - 21, 72 - 12 + 4, 90_deg), { max_speed: 110 });
    pros::delay(200);
    drive_controller->drive_time(127, 150);
    pros::delay(200);
    intake->set_state(IntakeState::DISABLED);

    // get stack
    drive_controller->boomerang(pos(144-48, 48), { backwards: true, threshold: 2, min_speed: 40 });
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 24, 48 + 3), { threshold: 3, min_speed: 80 });
    pros::delay(300);
    drive_controller->drive(-10, { min_speed: 60, chained: true });

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->boomerang(pos(144-24, 24));
    drive_controller->drive_time(-100, 50);
    drive_controller->face_angle(135_deg, { chained: true });
    drive_controller->face_angle(135_deg, { max_speed: 30 });
    pros::delay(50);
    drive_controller->drive_time(127, 200);
    drive_controller->drive_time(60, 500);
    drive_controller->drive_time(-100, 100);

    // touch ladder
    drive_controller->boomerang(pos(144-36, 36), { backwards: true, threshold: 2, min_speed: 60 } );
    drive_controller->face_angle(-45_deg, { threshold: 8_deg });
    drive_controller->drive_time(100, 500);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
}