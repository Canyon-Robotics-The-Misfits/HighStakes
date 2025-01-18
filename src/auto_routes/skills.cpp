#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(skills_start) {
    odometry->initialize(144 - 53 - 4, 13 + 1, 224_deg);

    clamp.retract();

    drive_controller->drive(-6, { min_speed: 60, chained: true });
    clamp.extend();
    pros::delay(100);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->face_angle(-100_deg, { threshold: 3_deg });
    drive_controller->drive_to(pose(76.8438 + 1, 13.1988 + 1, -160.197_deg), { r: 12, min_speed: 30 });
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    pros::delay(150);
    drive_controller->drive(-6, { min_speed: 80, chained: true });

}

AUTO_ROUTE(auto_routes::skills)
{
    RUN_AUTO(skills_start);

    // get rings for wall stake
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->boomerang(pos(144 - 48, 48));
    drive_controller->boomerang(pos(144 - 15, 72));
    pros::delay(600);

    // score wall stake
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->boomerang(pos(144 - 24, 72), { backwards: true, min_speed: 40, chained: true });
    intake->set_state(IntakeState::HOOD);
    pros::delay(200);
    drive_controller->drive_to(pose(144 - 13, 72, 90_deg));
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-6, { min_speed: 80, chained: true });

    // get next rings
    drive_controller->boomerang(pose(144 - 24, 72 + 24, 10_deg));
    auto drive_to_ring_stack = drive_controller->boomerang(pose(144 - 12, 144 - 24 - 4, 0_deg), { async: true });
    pros::delay(600);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_stack->await();
    pros::delay(200);
    drive_controller->drive(6, { min_speed: 40, chained: true});
    pros::delay(700);

    // wall stake again
    auto drive_to_wall_stake_2 = drive_controller->boomerang(pos(144 - 24, 72 - 6), { backwards: true, async: true });
    pros::delay(200);
    arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(100);
    intake->set_state(IntakeState::REVERSE);
    drive_to_wall_stake_2->await();
    drive_controller->drive_to(pose(144 - 12, 72, 90_deg), { min_speed: 25 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    
    // get rings near corner
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 24, 72 - 24));
    drive_controller->boomerang(pos(144 - 24, 24));
    pros::delay(100);
    drive_controller->boomerang(pos(144 - 24, 12 + 4), { min_speed: 30 });
    pros::delay(100);
    drive_controller->boomerang(pos(144 - 12, 24), { min_speed: 30 });
    drive_controller->boomerang(pos(144 - 24, 24), { backwards: true, threshold: 5 });
    drive_controller->face_point(lib15442c::Vec(144, 0), 180_deg, { threshold: 3_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 700);

    // get rings and next goal
    drive_controller->boomerang(pos(72, 72), { threshold: 3 });
    intake->set_state(IntakeState::DISABLED);
    drive_controller->boomerang(pose(48 + 12, 24 + 12, 180_deg), { backwards: true, lead: 0.4, min_speed: 60, chained: true });
    drive_controller->boomerang(pose(48, 24, 35_deg), { backwards: true, lead: 0.3, min_speed: 60, chained: true });
    clamp.extend();
}