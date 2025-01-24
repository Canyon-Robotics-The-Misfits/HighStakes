#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::red_rush_segment)
{
    odometry->initialize(144-9, 20, -20_deg);

    // goal rush
    doinker.extend();
    intake->set_state(IntakeState::HOOD);
    auto goal_rush = drive_controller->drive(37.5, { angle: odometry->get_rotation() - 5_deg, threshold: 1, async: true });
    arm->move_manual(127);
    pros::delay(200);
    arm->move_manual(0);
    goal_rush->await();
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
    drive_controller->face_point(lib15442c::Vec(144 - 48, 48-2), 180_deg - 20_deg, { threshold: 20_deg, min_speed: 60, chained: true });
    drive_controller->boomerang(pose(144 - 48, 48, 100_deg), { backwards: true, lead: 0.5, threshold: 7, min_speed: 60 });
    drive_controller->drive_time(-60, 150);
    clamp.extend();
    pros::delay(100);
}

AUTO_ROUTE(auto_routes::positive_red)
{
    RUN_AUTO(red_rush_segment);

    // score alliance stake
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+9-0.5, 16.5-4 -2, -153_deg), { min_speed: 25, async: true });
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    pros::delay(1300);
    intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drivetrain->move(0, -60);
    pros::delay(50);
    drivetrain->move(0, 60);
    pros::delay(50);
    drivetrain->move(0, 0);
    drive_controller->drive_time(-100, 200);

    // dropoff goal
    drive_controller->face_angle(45_deg, {threshold: 10_deg, chained: true });
    clamp.retract();

    // get rush goal
    drive_controller->drive(15, {min_speed: 60, chained: true });
    drive_controller->face_angle(-135_deg, {threshold: 3_deg, chained: true });
    drive_controller->boomerang(pos(144-24, 58 - 3), { backwards: true, threshold: 4, min_speed: 60 });
    clamp.extend();

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    drive_controller->boomerang(pose(144-24, 24, -147_deg), { lead: 0.4 });
    drive_controller->drive_time(-100, 50);
    intake->set_state(IntakeState::HOOD);
    // drive_controller->face_point(lib15442c::Vec(144, 0), 0_deg, { chained: true });
    // drive_controller->face_point(lib15442c::Vec(144, 0), 0_deg, { max_speed: 30 });
    drive_controller->face_angle(135_deg, { chained: true });
    drive_controller->face_angle(135_deg, { max_speed: 30 });
    pros::delay(50);
    drive_controller->drive_time(50, 1000);
    pros::delay(100);
    drive_controller->drive_time(-100, 300);
    
    // go to end spot
    drive_controller->boomerang(pos(144-24, 40), { backwards: true } );

}