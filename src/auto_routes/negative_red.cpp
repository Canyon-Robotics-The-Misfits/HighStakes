#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_red)
{
    odometry->initialize(41, 21, -15.8_deg);

    // ring rush
    intake->set_state(IntakeState::HOOD);
    doinker.extend();
    drive_controller->drive(41, { min_speed: 40, chained: true });
    // auto back_up_rush = drive_controller->drive(-20, { min_speed: 40, chained: true, async: true });
    // pros::delay(400);
    // back_up_rush->await();
    // doinker.retract();
    // pros::delay(50);

    // pick up goal
    // drive_controller->face_point(lib15442c::Vec(144 - 48, 48), 180_deg, { min_speed: 40, chained: true });
    auto drive_to_goal = drive_controller->boomerang(pos(48, 48 +1), { backwards: true, threshold: 3, min_speed: 40, chained: true, async: true });
    pros::delay(350);
    intake->set_state(IntakeState::DISABLED);
    drive_to_goal->await();
    clamp.extend();
    doinker.retract();
    intake->set_state(IntakeState::HOOD);
    pros::delay(100);

    // get stack
    drive_controller->face_angle(-98_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(24, 48), { threshold: 14, min_speed: 80 });
    intake->set_state(IntakeState::WALL_STAKE);
    pros::delay(450);
    auto get_second_line_ring = drive_controller->drive(4, { min_speed: 60, chained: true, async: true });
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    get_second_line_ring->await();

    // score wall stake
    // wall stake
    auto drive_to_wall_stake = drive_controller->drive_to(pose(10.5 -0.5, 72 - 9, -45_deg), { r: 4, timeout: 1500, max_speed: 100, min_speed: 40, async: true });
    pros::delay(200);
    arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(300);
    intake->set_state(IntakeState::DISABLED);
    intake_lift.extend();
    drive_to_wall_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    intake_lift.retract();

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->face_point(lib15442c::Vec(24, 28), 0_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(24, 28), { threshold: 8, min_speed: 60 });
    drive_controller->boomerang(pos(15, 15), { threshold: 2, min_speed: 40 });

    drive_controller->face_point(lib15442c::Vec(4, 0 +4), 5_deg, { threshold: 3_deg });
    pros::delay(50);
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive_time(50, 600);
    intake->set_state(IntakeState::HOOD);
    pros::delay(150);
    drive_controller->drive(-2.75, { min_speed: 40, chained: true });
    pros::delay(150);
    intake_lift.extend();
    pros::delay(100);
    drive_controller->drive_time(60, 250);
    pros::delay(150);
    intake_lift.retract();
    intake->set_state(IntakeState::WALL_STAKE);
    pros::delay(150);
    auto backup_from_corner = drive_controller->drive(-8, { min_speed: 40, chained: true, async: true });
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    backup_from_corner->await();

    // get middle ring
    auto drive_to_middle_ring = drive_controller->drive_to(pose(72 - 8, 24 -2, 90_deg), { min_speed: 40, async: true });
    pros::delay(800);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    pros::delay(100);
    intake_lift.extend();
    drive_to_middle_ring->await();
    pros::delay(50);
    intake_lift.retract();
    pros::delay(200);
    drive_controller->drive(-6, { min_speed: 60, chained: true });

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::HOOD);
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72-8 +1.5 - 2, 14 +0.5, 148_deg), { threshold: 1, timeout: 1000, min_speed: 40, async: true });
    WAIT_UNTIL(odometry->get_pose().vec().distance_to(lib15442c::Vec(72, 12)) < 14 || !drive_to_alliance_stake->is_running());
    intake->set_state(IntakeState::DISABLED);
    intake_lift.extend();
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-8, { min_speed: 100, chained: true });
    intake_lift.retract();
    intake->set_state(IntakeState::HOOD);

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    drive_controller->boomerang(pos(47, 47), { backwards: true, threshold: 5, min_speed: 60 });
    drive_controller->face_angle(45_deg, { min_speed: 30, chained: true });
    drive_controller->drive_time(60, 200);

    // // get ladder ring
    // drive_controller->boomerang(pos(144 - 48 - 6, 48 - 6), { backwards: true, threshold: 12, min_speed: 80 });
    // // drive_controller->face_point(lib15442c::Vec(72, 72), 0_deg, { min_speed: 60, chained: true });
    // drive_controller->drive_to(pose(72 + 15, 72 - 15, -45_deg), { r: 17, min_speed: 60 });
}