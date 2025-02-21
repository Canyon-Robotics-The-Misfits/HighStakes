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
    drive_controller->drive(30, { threshold: 0, min_speed: 40, chained: true });
    doinker.retract();
    drive_controller->turn(-5_deg, { timeout: 100, min_speed: 50, chained: true });
    drive_controller->drive_time(-127, 250, { ramp_up: true, ramp_speed:  127.0/0.5 });
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive(-11, { min_speed: 80, chained: true });
    doinker.extend();
    pros::delay(50);
    drive_controller->drive(-3, { min_speed: 80, chained: true });
    doinker.retract();

    // get next goal
    drive_controller->face_point(lib15442c::Vec(144 - 36, 48), 180_deg, { threshold: 5_deg });
    drive_controller->boomerang(pos(144 - 32, 48), { backwards: true, threshold: 10, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 5, min_speed: 60 });
    clamp.extend();
    pros::delay(100);
}

AUTO_ROUTE(auto_routes::positive_red)
{
    RUN_AUTO(red_rush_segment);

    // get ring with doinker
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_to(pose(72 + 18, 29, -125_deg), { r: 8, threshold: 2, min_speed: 35 });
    drive_controller->face_angle(-125_deg, { threshold: 3_deg });
    doinker.extend();
    pros::delay(250);
    drive_controller->face_angle(-160_deg, { threshold: 3_deg, chained: true });
    doinker.retract();
    pros::delay(50);
    drive_controller->face_angle(-150_deg, { threshold: 5_deg, chained: true });

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::HOOD);
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+8, 15, -148_deg), { threshold: 1, timeout: 1000, min_speed: 30, async: true });
    WAIT_UNTIL(odometry->get_pose().vec().distance_to(lib15442c::Vec(72, 12)) < 16 || !drive_to_alliance_stake->is_running());
    intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 60, chained: true });

    // get corner
    arm->set_state(ArmState::ALLIANCE_STAKE);
    auto drive_to_corner = drive_controller->boomerang(pose(144 - 18, 18, 70_deg), { lead: 0.4, threshold: 8, angle_priority_threshold: 10, min_speed: 60, async: true });
    intake->set_state(IntakeState::HOOD);
    pros::delay(1000);
    intake->set_state(IntakeState::DISABLED);
    drive_to_corner->await();
    drive_controller->face_point(lib15442c::Vec(144, 0), 0_deg, { chained: true });

    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_time(60, 500);
    auto leave_corner = drive_controller->drive(-18, { threshold: 0, min_speed: 40, chained: true, async: true });
    pros::delay(400);
    intake->set_state(IntakeState::DISABLED);
    leave_corner->await();

    // dropoff goal
    drive_controller->face_angle(-45_deg, { threshold: 15_deg, chained: true });
    clamp.retract();
    pros::delay(100);

    // get other goal
    drive_controller->drive(4, { min_speed: 80, chained: true });
    drive_controller->face_point(lib15442c::Vec(144 - 18, 72 - 12), 180_deg, { threshold: 5_deg, chained: true });
    drive_controller->boomerang(pos(144 - 18, 72 - 12), { backwards: true, threshold: 4, min_speed: 60 });
    clamp.extend();

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 48, 48), { threshold: 5 , min_speed: 6 });
    drive_controller->face_angle(-45_deg, { min_speed: 40, chained: true });
    drive_controller->drive_time(80, 400);

}