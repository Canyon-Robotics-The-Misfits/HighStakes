#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(skills_start) {
    odometry->initialize(144 - 53 - 4 + 2, 13 + 1 + 3, 224_deg);

    clamp.retract();

    drive_controller->drive(-6, { min_speed: 60, chained: true });
    clamp.extend();
    pros::delay(100);
    drive_controller->face_angle(-100_deg, { threshold: 3_deg });
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->drive_to(pose(76.8438 + 1 + 2, 13.1988 + 1 + 3, -160.197_deg), { r: 12, min_speed: 30 });
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    pros::delay(150);
    drive_controller->drive(-6, { min_speed: 80, chained: true });

}

AUTO_ROUTE(auto_routes::skills)
{
    /*
    odometry->initialize(0, 0, 0_deg);

    auto trajectory_builder = lib15442c::TrajectoryBuilder({ point: lib15442c::Vec(0, 0), tangent: lib15442c::Vec(0, 48) });
    trajectory_builder.append_hermite({ point: lib15442c::Vec(24, 48), tangent: lib15442c::Vec(0, 48) });

    auto trajectory = trajectory_builder.compute(config::TRAJECTORY_CONSTRAINTS, -1, true);
    trajectory.debug_log();

    auto ramsete = lib15442c::RAMSETE(trajectory, 0.0013, 0.7);

    ramsete.execute(drivetrain, odometry);

    return;
    */
    RUN_AUTO(skills_start);
    
    pros::Distance distance_left = pros::Distance(11);
    pros::Distance distance_right = pros::Distance(16);
    double mm_to_in = 0.0394;

    // get rings for wall stake
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->boomerang(pos(144 - 48, 48));
    auto pickup_second_ring = drive_controller->boomerang(pos(144 - 15, 72), { async: true });
    pros::delay(800);
    intake->set_state(IntakeState::HOOD);
    pickup_second_ring->await();
    pros::delay(100);

    // score wall stake
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->drive(-6, { min_speed: 60, chained: true });
    drive_controller->drive_to(pose(144 - 13, 72, 90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-6, { min_speed: 80, chained: true });

    // get next rings
    drive_controller->boomerang(pose(144 - 24, 72 + 24 - 4, 10_deg));
    auto drive_to_ring_stack = drive_controller->boomerang(pose(144 - 12, 144 - 24 - 5, 0_deg), { async: true });
    pros::delay(400);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_stack->await();
    pros::delay(200);
    drive_controller->drive(8, { min_speed: 40, chained: true});
    pros::delay(350);
    odometry->set_x(144 - (distance_right.get() * mm_to_in + 6) - 3);
    pros::delay(350);

    // wall stake again
    auto drive_to_wall_stake_2 = drive_controller->boomerang(pos(144 - 24, 72 - 6), { backwards: true, async: true });
    pros::delay(200);
    arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(100);
    intake->set_state(IntakeState::REVERSE);
    drive_to_wall_stake_2->await();
    drive_controller->drive_to(pose(144 - 12 - 3, 72, 90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    
    // get rings near corner
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 24, 72 - 24));
    drive_controller->boomerang(pos(144 - 24, 24));
    pros::delay(100);
    drive_controller->boomerang(pos(144 - 24, 12 + 5), { min_speed: 30 });
    pros::delay(100);
    drive_controller->boomerang(pos(144 - 12, 24), { min_speed: 30 });
    drive_controller->boomerang(pos(144 - 24, 24), { backwards: true, threshold: 5, min_speed: 60 });
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