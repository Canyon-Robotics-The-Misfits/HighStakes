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
    drive_controller->drive(29.5, { threshold: 0, min_speed: 50, chained: true });
    doinker.retract();
    drive_controller->drive_time(-127, 250, { ramp_up: true, ramp_speed:  127.0/0.5 });
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive(-11, { min_speed: 80, chained: true });
    doinker.extend();
    pros::delay(150);
    // drive_controller->drive_time(40, 100);
    drive_controller->drive(-3, { min_speed: 80, chained: true });
    doinker.retract();

    // get next goal
    drive_controller->face_point(lib15442c::Vec(144 - 36, 48), 180_deg, { threshold: 5_deg });
    drive_controller->boomerang(pos(144 - 32, 48), { backwards: true, threshold: 15, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 5, min_speed: 60 });
    clamp.extend();
    pros::delay(100);
}

AUTO_ROUTE(auto_routes::positive_red)
{
    RUN_AUTO(red_rush_segment);

    // get ring with lifting intake
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_point(lib15442c::Vec(72, 24), 0_deg, { min_speed: 40, chained: true });
    arm->set_state(ArmState::ALLIANCE_STAKE);
    auto drive_to_middle_ring = drive_controller->boomerang(pos(72, 24), { threshold: 6, min_speed: 35, async: true });
    pros::delay(500);
    intake_lift.extend();
    drive_to_middle_ring->await();
    pros::delay(50);
    intake_lift.retract();
    pros::delay(100);
    drive_controller->drive(-6, { max_speed: 60, min_speed: 60, chained: true });
    pros::delay(50);

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::HOOD);
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+8 -1.5, 17, -148_deg), { threshold: 1, min_speed: 30, async: true });
    WAIT_UNTIL(odometry->get_pose().vec().distance_to(lib15442c::Vec(72, 12)) < 16 || !drive_to_alliance_stake->is_running());
    intake->set_state(IntakeState::DISABLED);
    intake_lift.extend();
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 60, chained: true });
    intake_lift.retract();

    // get corner
    arm->set_state(ArmState::ALLIANCE_STAKE);
    auto drive_to_corner = drive_controller->boomerang(pose(144 - 17.5, 17.5, 65_deg), { lead: 0.4, threshold: 8, angle_priority_threshold: 10, min_speed: 60, async: true });
    intake->set_state(IntakeState::HOOD);
    pros::delay(1000);
    intake->set_state(IntakeState::DISABLED);
    drive_to_corner->await();
    drive_controller->face_point(lib15442c::Vec(144 - 4, 4), 0_deg, { chained: true });

    intake->set_state(IntakeState::HOOD);
    pros::delay(50);
    drive_controller->drive_time(60, 400);
    pros::delay(150);
    drive_controller->drive(-2.5, { min_speed: 40, chained: true });
    pros::delay(150);
    intake_lift.extend();
    pros::delay(100);
    drive_controller->drive_time(60, 250);
    pros::delay(150);
    intake_lift.retract();
    pros::delay(150);
    arm->set_state(ArmState::LOAD);
    auto backup_from_corner = drive_controller->drive(-10, { max_speed: 80, min_speed: 80, chained: true, async: true });
    pros::delay(200);
    intake->set_state(IntakeState::DISABLED);
    backup_from_corner->await();

    // dropoff goal
    drive_controller->face_angle(-45_deg, { threshold: 15_deg, chained: true });
    clamp.retract();
    pros::delay(100);

    // get other goal
    drive_controller->drive(5, { min_speed: 80, chained: true });
    drive_controller->face_point(lib15442c::Vec(144 - 18, 72 - 16), 180_deg, { threshold: 10_deg, min_speed: 60, chained: true });
    arm->set_state(ArmState::LADDER_TOUCH);
    drive_controller->boomerang(pos(144 - 18, 72 - 16), { backwards: true, threshold: 4, min_speed: 60 });
    clamp.extend();

    // touch ladder
    intake->set_state(IntakeState::HOOD);
    intake_lift.extend();
    drive_controller->boomerang(pos(144 - 48, 48 +4), { threshold: 5, min_speed: 60 });
    drive_controller->face_angle(-45_deg, { min_speed: 40, chained: true });
    // drive_controller->drive_time(80, 200);
    intake->set_state(IntakeState::DISABLED);

}