#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::blue_rush_segment)
{
    odometry->initialize(38 +1.5, 24, -20_deg);

    // goal rush
    doinker.extend();
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(38.5, { angle: odometry->get_rotation() - 5_deg, threshold: 1 });
    doinker.retract();
    intake->set_state(IntakeState::DISABLED);
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    drive_controller->drive_time(-127, 250);
    drive_controller->drive_time(-80, 200);
    doinker.extend();
    pros::delay(100);
    drive_controller->drive_time(40, 50);
    pros::delay(100);
    drive_controller->drive(-3, { threshold: 1, min_speed: 40, chained: true });
    doinker.retract();

    // get next goal
    drive_controller->face_point(lib15442c::Vec(48, 48+4), 180_deg, { threshold: 5_deg, min_speed: 60, chained: true });
    drive_controller->boomerang(pos(48, 48+4), { backwards: true, threshold: 7, min_speed: 60 });
    clamp.extend();
    pros::delay(100);
}

AUTO_ROUTE(auto_routes::positive_blue)
{
    RUN_AUTO(auto_routes::blue_rush_segment);

    // score alliance stake
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72-9+4, 16.5 + 1, 148_deg), { min_speed: 25, async: true });
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    pros::delay(1300);
    intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive_time(-100, 300);

    // dropoff goal
    drive_controller->face_angle(-10_deg, {threshold: 10_deg, chained: true });
    clamp.retract();

    // get rush goal
    drive_controller->drive(15, {min_speed: 60, chained: true });
    drive_controller->face_angle(-60_deg + 180_deg, {threshold: 3_deg, chained: true });
    drive_controller->boomerang(pos(20, 65), { backwards: true, threshold: 6, min_speed: 60 });
    clamp.extend();

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    drive_controller->boomerang(pose(18, 18, 180_deg), { lead: 0.4, threshold: 1.5, min_speed: 30 } );
    drive_controller->drive_time(-100, 150);
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_point(lib15442c::Vec(0, 0));
    pros::delay(50);
    drive_controller->drive_time(60, 800);
    drive_controller->drive_time(-100, 300);

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    drive_controller->face_angle(45_deg, { threshold: 20_deg });
    intake->set_state(IntakeState::DISABLED);
    drive_controller->boomerang(pose(48, 48, 45_deg), { threshold: 2, min_speed: 60 } );
    drive_controller->drive_time(80, 200);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
    
    // // go to end spot (replace touch ladder)
    // drive_controller->boomerang(pos(24, 40), { backwards: true } );
}