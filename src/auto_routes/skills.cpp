#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

AUTO_ROUTE(auto_routes::skills_start_segment) {
    odometry->initialize(83 +1, 17 + 5, 250_deg);

    clamp.retract();

    drive_controller->drive(-4, { min_speed: 60, chained: true });
    clamp.extend();
    pros::delay(100);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->drive_to(pose(76.8438 + 2 -0.5, 13.1988 + 1.5 + 2, -160.197_deg), { r: 10, min_speed: 30 });
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
    RUN_AUTO(skills_start_segment);
    
    pros::Distance distance_left = pros::Distance(11);
    pros::Distance distance_right = pros::Distance(16);
    // pros::Distance distance_front = pros::Distance(5);
    double mm_to_in = 0.0394;

    // get rings for wall stake
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->boomerang(pos(144 - 48, 48));
    pros::delay(300);

    // score wall stake
    auto wall_stake_1 = drive_controller->drive_to(pose(144 - 14, 72 +2, 90_deg), { max_speed: 80, min_speed: 30, async: true });
    pros::delay(700);
    intake->set_state(IntakeState::HOOD);
    arm->set_state(ArmState::NEUTRAL_STAKE);
    wall_stake_1->await();
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-8, { min_speed: 80, chained: true });

    // get next rings
    drive_controller->boomerang(pose(144 - 24 - 3, 72 + 24 - 4, 10_deg), { min_speed: 60 });
    auto drive_to_ring_stack = drive_controller->boomerang(pose(144 - 12 -1, 144 - 24 - 5, 0_deg), { async: true });
    pros::delay(600);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_stack->await();
    pros::delay(200);
    drive_controller->drive(9, { chained: true});
    pros::delay(200);
    odometry->set_x(144 - (distance_right.get() * mm_to_in + 6) - 3);
    // odometry->set_y(144 - (distance_front.get() * mm_to_in + 6.5));
    pros::delay(200);

    // wall stake again
    drive_controller->boomerang(pos(144 - 24, 72 - 6), { backwards: true });
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->drive_to(pose(144 - 14, 72 +2, 90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    
    // get rings near corner; dropoff goal
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_point(lib15442c::Vec(144 - 24, 72 - 24), 0_deg, { threshold: 15_deg, chained: true });
    drive_controller->boomerang(pos(144 - 24, 72 - 24), { min_speed: 60 });
    pros::delay(50);
    drive_controller->boomerang(pos(144 - 24 - 2, 24 + 5));
    pros::delay(50);
    drive_controller->boomerang(pos(144 - 24 - 2, 12 + 5), { min_speed: 30 });
    pros::delay(50);
    drive_controller->face_angle(45_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(144 - 12 -1, 24), { min_speed: 50 });
    drive_controller->boomerang(pos(144 - 18, 18), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_point(lib15442c::Vec(144, 0), 180_deg, { threshold: 10_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 400);

    // ---------- TRANSITION ---------- //

    // get rings and next goal
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(72, 72));
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->face_angle(-135_deg, { threshold: 5_deg, chained: true });

    auto drive_to_ring_before_goal_2 = drive_controller->boomerang(pos(48, 48), { threshold: 5, min_speed: 60, async: true });
    pros::delay(700);
    intake->set_state(IntakeState::HOOD);
    drive_to_ring_before_goal_2->await();

    auto turn_to_goal_2 = drive_controller->face_angle(-30_deg, { threshold: 10_deg, chained: true, async: true });
    pros::delay(350);
    intake->set_state(IntakeState::DISABLED);
    turn_to_goal_2->await();
    drive_controller->boomerang(pose(48, 24, -30_deg + 180_deg), { backwards: true, lead: 0.7, threshold: 8, min_speed: 60, chained: true,  });

    drive_controller->drive_time(-60, 100);
    clamp.extend();
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);

    // ---------- START SIDE 2 ---------- //

    // get rings for wall stake
    auto drive_to_ring_before_wall_stake = drive_controller->boomerang(pose(14, 72, -75_deg), { async: true });
    pros::delay(500);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_before_wall_stake->await();
    pros::delay(200);

    // score wall stake
    drive_controller->drive(-8, { min_speed: 60, chained: true });
    pros::delay(400);
    arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(500);
    drive_controller->drive_to(pose(14 -1, 72, -90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-8, { min_speed: 80, chained: true });

    // get next rings
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pose(24 + 3, 72 + 24 - 4, -10_deg), { min_speed: 60 });
    auto drive_to_ring_stack_2 = drive_controller->boomerang(pose(12 +1, 144 - 24 - 5, 0_deg), { async: true });
    pros::delay(600);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_stack_2->await();
    pros::delay(200);
    drive_controller->drive(9, { chained: true});
    pros::delay(200);
    odometry->set_x(distance_left.get() * mm_to_in + 6 +1);
    pros::delay(200);

    // wall stake again
    drive_controller->boomerang(pos(24, 72 - 6), { backwards: true });
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->drive_to(pose(14, 72 +2, -90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    
    // get rings near corner; dropoff goal
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_point(lib15442c::Vec(24, 72 - 24), 0_deg, { threshold: 15_deg, chained: true });
    drive_controller->boomerang(pos(24, 72 - 24), { min_speed: 60 });
    drive_controller->boomerang(pos(24 + 1, 24 + 5));
    // pros::delay(200);
    drive_controller->boomerang(pos(24, 12 + 5), { min_speed: 30 });
    pros::delay(50);
    drive_controller->face_angle(-45_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(12 +1, 24), { min_speed: 50 });
    drive_controller->boomerang(pos(18, 18), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_point(lib15442c::Vec(0, 0), 180_deg, { threshold: 10_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 400);

    // ---------- END ---------- //

    // go through ring and get 3rd goal
    drive_controller->boomerang(pos(30, 72), { threshold: 5, min_speed: 80 });
    drive_controller->boomerang(pos(48, 72 + 24));
    auto face_third_goal = drive_controller->face_angle(-135_deg, { threshold: 20_deg, chained: true, async: true });
    drive_controller->boomerang(pos(72, 144 - 24), { backwards: true, threshold: 8, min_speed: 60 });
    clamp.extend();
    drive_controller->face_angle(-90_deg);
    pros::delay(50);
    odometry->set_y(144 - (distance_right.get() * mm_to_in + 6) - 3);
    intake->set_state(IntakeState::HOOD);

    // get ring for 3rd goal and alliance stake
    drive_controller->boomerang(pose(24, 144 - 24, 80_deg), { threshold: 2 });
    drive_controller->drive(-10, { min_speed: 30, chained: true });
    drive_controller->boomerang(pose(24, 144 - 12, -45_deg), { threshold: 8 });
    intake->set_state(IntakeState::WALL_STAKE);
    pros::delay(400);
    drive_controller->drive(6, { min_speed: 40, chained: true});
    drive_controller->drive(-4, { min_speed: 40, chained: true});
    pros::delay(350);

    // dropoff goal then alliance stake
    drive_controller->boomerang(pose(72 + 12, 144 - 30, -110_deg), { backwards: true, lead: 0.8 });
    clamp.retract();
    pros::delay(50);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->drive_to(pose(72, 144 - 13, 0_deg));
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    pros::delay(100);
    drive_controller->drive(-4, { min_speed: 60, chained: true });

    // push in corner goal
    drive_controller->boomerang(pos(72 - 24, 144 - 12), { threshold: 8, min_speed: 80 });
    drive_controller->boomerang(pos(24, 144 - 12));
}