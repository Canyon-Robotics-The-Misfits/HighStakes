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
    drive_controller->drive(-8, { max_speed: 50, min_speed: 40, chained: true });
    doinker.retract();
    pros::delay(50);
    drive_controller->face_angle(70_deg, { chained: true });
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(9, { min_speed: 60, chained: true });
    pros::delay(300);
    intake->set_state(IntakeState::DISABLED);
    drive_controller->face_point(lib15442c::Vec(72, 0), 0_deg, { chained: true });
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72-8 -1, 12, 145_deg), { r: 2, threshold: 1, timeout: 1000, min_speed: 30, async: true });
    // WAIT_UNTIL(odometry->get_pose().vec().distance_to(lib15442c::Vec(72, 12)) < 16 || !drive_to_alliance_stake->is_running());
    // intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 60, chained: true });

    // grab goal
    // drive_controller->boomerang(pose(48 -2, 48, -15_deg + 180_deg), { backwards: true, lead: 0.7, threshold: 3, angle_priority_threshold: 12, min_speed: 60 });
    drive_controller->boomerang(pos(48 -2, 48), { backwards: true, threshold: 3, min_speed: 60 });
    drive_controller->drive_time(-60, 50);
    clamp.extend();
    intake->set_state(IntakeState::REVERSE);
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    
    // get line rings
    drive_controller->drive_to(pose(26, 72 - 12 + 2, -90_deg), { max_speed: 110, min_speed: 40 });
    pros::delay(400);
    drive_controller->drive_time(60, 400);
    pros::delay(100);

    // get stack
    drive_controller->boomerang(pos(48, 48), { backwards: true, threshold: 2, min_speed: 40 });
    drive_controller->boomerang(pos(24, 48), { threshold: 3, min_speed: 80 });
    pros::delay(300);
    drive_controller->drive(-10, { min_speed: 60, chained: true });

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->boomerang(pos(18 +1, 18 -1), { threshold: 2 });
    drive_controller->face_point(lib15442c::Vec(0, 0), 5_deg);
    pros::delay(50);
    drive_controller->drive_time(60, 800);
    pros::delay(100);
    drive_controller->drive_time(-100, 200);

    // // touch ladder
    // arm->set_state(ArmState::LADDER_TOUCH);
    // drive_controller->boomerang(pos(36, 36), { backwards: true, threshold: 2, min_speed: 60 } );
    // drive_controller->face_angle(40_deg, { threshold: 5_deg });
    // drive_controller->drive_time(60, 700);
    // arm->set_state(ArmState::DISABLED);
    // // drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

    // end position
    drive_controller->face_angle(90_deg);
    intake->set_state(IntakeState::DISABLED);
    drive_controller->boomerang(pos(72, 24), { threshold: 8, max_speed: 100, min_speed: 60 }); 
    drive_controller->boomerang(pos(144 - 48, 48), { max_speed: 60 });
}