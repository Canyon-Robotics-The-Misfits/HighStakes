#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::red_rush_segment)
{
    odometry->initialize(144-9 -2, 21, 341_deg);

    // goal rush
    doinker.extend();
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(31, { threshold: 0, min_speed: 40, chained: true });
    doinker.retract();
    pros::delay(20);
    drive_controller->drive_time(-127, 250, { ramp_up: true, ramp_speed:  127.0/0.5 });
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive(-11, { min_speed: 80, chained: true });
    doinker.extend();
    drive_controller->drive(-3, { min_speed: 80, chained: true });
    doinker.retract();

    // get next goal
    drive_controller->face_point(lib15442c::Vec(144 - 36, 48), 180_deg, { threshold: 5_deg });
    drive_controller->boomerang(pos(144 - 32, 48), { backwards: true, threshold: 8, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 5, min_speed: 60 });
    clamp.extend();
    pros::delay(100);
}

AUTO_ROUTE(auto_routes::positive_red)
{
    RUN_AUTO(red_rush_segment);

    // get ring with doinker
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_to(pose(72 + 18, 29, -125_deg), { threshold: 2, min_speed: 30 });
    drive_controller->face_angle(-125_deg, { threshold: 3_deg });
    doinker.extend();
    pros::delay(150);
    drive_controller->face_angle(-160_deg, { chained: true });
    doinker.retract();
    pros::delay(50);
    drive_controller->face_angle(-150_deg, { chained: true });

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::HOOD);
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+7, 15, -148_deg), { threshold: 1, timeout: 1000, min_speed: 30, async: true });
    WAIT_UNTIL(odometry->get_pose().vec().distance_to(lib15442c::Vec(72, 12)) < 16 || !drive_to_alliance_stake->is_running());
    intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 60, chained: true });
    intake->set_state(IntakeState::HOOD);

    // get corner
    arm->set_state(ArmState::ALLIANCE_STAKE);
    auto drive_to_corner = drive_controller->boomerang(pose(144 - 24, 16, 110_deg), { lead: 0.5, threshold: 3, angle_priority_threshold: 6, min_speed: 100, async: true });
    WAIT_UNTIL(odometry->get_x() > 144 - 44 || !drive_to_corner->is_running());
    doinker.extend();
    intake->set_state(IntakeState::REVERSE);
    drive_to_corner->await();
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive(6, { min_speed: 100, chained: true });
    pros::delay(200);
    drive_controller->turn(-10_deg, { min_speed: 80, chained: true });
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_angle(-35_deg, { max_speed: 80 });
    arm->set_state(ArmState::LOAD);
    drive_controller->drive(16, { min_speed: 60, chained: true });

    // dropoff goal
    doinker.retract();
    pros::delay(600);
    drive_controller->drive(-8, { min_speed: 60, chained: true });
    clamp.retract();

    // get other goal
    drive_controller->drive(10, { min_speed: 80, chained: true });
    drive_controller->face_point(lib15442c::Vec(144 - 18, 72 - 12), 180_deg, { threshold: 5_deg, chained: true });
    drive_controller->boomerang(pos(144 - 18, 72 - 12), { backwards: true, threshold: 5, min_speed: 60 });
    clamp.extend();

    // get ring under ladder
    intake->set_state(IntakeState::DISABLED);
    drive_controller->boomerang(pos(144 - 48, 48), { threshold: 15, min_speed: 60 });
    drive_controller->face_angle(-42_deg, { threshold: 5_deg, chained: true });
    drive_controller->boomerang(pose(72 + 11, 72 - 10, -45_deg), { min_speed: 30 });
    doinker.extend();
    pros::delay(50);
    drive_controller->drive(-12, { min_speed: 60, chained: true});
    doinker.retract();

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    intake->set_state(IntakeState::DISABLED);
    drive_controller->boomerang(pose(144 - 48, 48, -45_deg), { lead: 0.4, threshold: 2, min_speed: 60 } );
    // drive_controller->drive_time(80, 100);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
    
    // // go to end spot (replace touch ladder)
    // drive_controller->boomerang(pos(144-24, 40), { backwards: true } );

}