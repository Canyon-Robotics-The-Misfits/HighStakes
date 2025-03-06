#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::negative_blue)
{
    odometry->initialize(144-40, 21, 13.5_deg);

    // ring rush
    intake->set_state(IntakeState::HOOD);
    doinker.extend();
    drive_controller->drive(40, { min_speed: 40, chained: true });
    auto back_up_rush = drive_controller->drive(-20, { min_speed: 40, chained: true, async: true });
    pros::delay(400);
    intake->set_state(IntakeState::DISABLED);
    back_up_rush->await();
    doinker.retract();
    pros::delay(50);

    // pick up goal
    drive_controller->face_point(lib15442c::Vec(144 - 48, 48), 180_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true, threshold: 5, min_speed: 80, chained: true });
    drive_controller->drive_time(-60, 150);
    clamp.extend();
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);

    // get stack
    drive_controller->boomerang(pos(144 - 24, 48 +3), { threshold: 3, min_speed: 80 });
    drive_controller->drive(3, { min_speed: 80, chained: true });

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->face_point(lib15442c::Vec(144 - 24, 20), 0_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 22, 24), { threshold: 8, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 18 +1, 18 +1), { threshold: 2, min_speed: 30 });

    drive_controller->face_point(lib15442c::Vec(144, 0), -10_deg, { threshold: 3_deg });
    pros::delay(50);
    drive_controller->drive_time(60, 450);
    pros::delay(150);
    drive_controller->drive(-2.5, { min_speed: 40, chained: true });
    pros::delay(150);
    intake_lift.extend();
    pros::delay(100);
    drive_controller->drive_time(60, 200);
    pros::delay(250);
    intake_lift.retract();
    pros::delay(100);
    arm->set_state(ArmState::LOAD);
    auto back_up_from_corner = drive_controller->drive(-10, { min_speed: 80, chained: true, async: true });
    pros::delay(100);
    intake->set_state(IntakeState::WALL_STAKE);
    back_up_from_corner->await();

    // get middle ring
    auto drive_to_middle_ring = drive_controller->drive_to(pose(72 + 8, 24, -90_deg), { min_speed: 40, async: true });
    pros::delay(900);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    pros::delay(100);
    intake_lift.extend();
    intake->set_state(IntakeState::HOOD);
    drive_to_middle_ring->await();
    pros::delay(50);
    intake_lift.retract();
    drive_controller->drive(-6, { min_speed: 40, chained: true });

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::HOOD);
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+8 -1.5 + 2, 14 +1, -148_deg), { threshold: 1, timeout: 1000, min_speed: 40, async: true });
    WAIT_UNTIL(odometry->get_pose().vec().distance_to(lib15442c::Vec(72, 12)) < 16 || !drive_to_alliance_stake->is_running());
    intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 60, chained: true });
}