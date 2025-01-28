#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

double distance_left()
{
    pros::Distance sensor = pros::Distance(11);

    return sensor.get() * 0.0361882 + 6.63209; // line of best fit to convert millimeters sensed to robot displacement in inches
}

double distance_right()
{
    pros::Distance sensor = pros::Distance(16);

    return sensor.get() * 0.038347 + 5.9759; // line of best fit to convert millimeters sensed to robot displacement in inches
}

double distance_front()
{
    pros::Distance sensor = pros::Distance(3);

    return sensor.get() * 0.0389846 + 6.03126; // line of best fit to convert millimeters sensed to robot displacement in inches
}

double distance_back()
{
    pros::Distance sensor = pros::Distance(5);

    return sensor.get() * 0.0395116 + 5.06893; // line of best fit to convert millimeters sensed to robot displacement in inches
}

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
    RUN_AUTO(skills_start_segment);

    // get rings for wall stake
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->boomerang(pos(144 - 48, 48), { threshold: 3, min_speed: 60 });
    drive_controller->drive_time(60, 100);
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
    drive_controller->boomerang(pose(144 - 24, 72 + 24 - 4, 10_deg), { min_speed: 60 });
    auto drive_to_ring_stack_2 = drive_controller->boomerang(pose(144 - (12 -1), 144 - 24 - 5, 0_deg), { min_speed: 30, async: true });
    pros::delay(800);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_stack_2->await();
    pros::delay(300);
    drive_controller->drive(10, { angle: 0_deg, min_speed: 30, chained: true});
    pros::delay(150);
    odometry->set_x(142 - distance_right()); // reset odom w/ distance sensors
    odometry->set_y(142 - distance_front());

    // wall stake again
    drive_controller->boomerang(pos(144 - 24, 72 - 6), { backwards: true });
    arm->set_state(ArmState::NEUTRAL_STAKE);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_to(pose(144 - 14 +2, 72, 90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    
    // get rings near corner; dropoff goal
    drive_controller->face_point(lib15442c::Vec(144 - 24, 72 - 24), 0_deg, { threshold: 15_deg, chained: true });
    drive_controller->boomerang(pos(144 - 24, 72 - 24), { threshold: 5, min_speed: 110 });
    drive_controller->boomerang(pos(144 - 24, 12 - 1), { min_speed: 25 });
    pros::delay(50);
    odometry->set_x(142 - distance_left()); // reset odom w/ distance sensors
    drive_controller->face_angle(45_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(144 - 12 -1, 24), { min_speed: 50 });
    drive_controller->boomerang(pos(144 - 18, 18), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_point(lib15442c::Vec(144, 0), 180_deg, { threshold: 10_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 300);

    // ---------- TRANSITION ---------- //

    // odometry->initialize(72 - 7.5, 72 - 8 + 2, 180_deg);

    // get rings and next goal
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(72, 72), { threshold: 4 });
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->face_angle(-135_deg, { threshold: 5_deg, chained: true });

    auto drive_to_ring_before_goal_2 = drive_controller->boomerang(pos(48, 48), { threshold: 5, min_speed: 60, async: true });
    pros::delay(700);
    intake->set_state(IntakeState::HOOD);
    drive_to_ring_before_goal_2->await();

    auto turn_to_goal_2 = drive_controller->face_angle(-30_deg, { threshold: 10_deg, chained: true, async: true });
    pros::delay(200);
    intake->set_state(IntakeState::DISABLED);
    turn_to_goal_2->await();
    drive_controller->boomerang(pose(48 -3, 24, -30_deg + 180_deg), { backwards: true, lead: 0.7, threshold: 8, min_speed: 60, chained: true,  });

    drive_controller->drive_time(-60, 100, { angle: 0_deg });
    clamp.extend();
    drive_controller->drive_time(-60, 100, { angle: 0_deg });
    odometry->set_y(distance_back());
    intake->set_state(IntakeState::HOOD);

    // ---------- START SIDE 2 ---------- //

    // get rings for wall stake
    auto drive_to_ring_before_wall_stake = drive_controller->boomerang(pose(13, 72, -75_deg), { async: true });
    pros::delay(500);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_before_wall_stake->await();
    pros::delay(200);

    // score wall stake
    drive_controller->drive(-8, { min_speed: 60, chained: true });
    pros::delay(300);
    intake->set_state(IntakeState::HOOD);
    arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(500);
    drive_controller->drive_to(pose(12, 72 -3, -90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-8, { min_speed: 80, chained: true });

    // get next rings
    drive_controller->boomerang(pose(24, 72 + 24 - 4, -10_deg), { min_speed: 60 });
    auto drive_to_ring_stack_1 = drive_controller->boomerang(pose(12 -1, 144 - 24 - 5, 0_deg), { min_speed: 30, async: true });
    pros::delay(800);
    intake->set_state(IntakeState::WALL_STAKE);
    drive_to_ring_stack_1->await();
    pros::delay(300);
    drive_controller->drive(9, { angle: 0_deg, min_speed: 30, chained: true});
    pros::delay(150);
    odometry->set_x(distance_left()); // reset odom w/ distance sensors
    odometry->set_y(142 - distance_front());

    // wall stake again
    drive_controller->boomerang(pos(24, 72 - 6), { backwards: true });
    arm->set_state(ArmState::NEUTRAL_STAKE);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_to(pose(12, 72, -90_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    
    // get rings near corner; dropoff goal
    drive_controller->face_point(lib15442c::Vec(24, 72 - 24), 0_deg, { threshold: 15_deg, chained: true });
    drive_controller->boomerang(pos(24, 72 - 24), { threshold: 5, min_speed: 110 });
    drive_controller->boomerang(pos(24, 12 - 1), { min_speed: 30 });
    pros::delay(50);
    drive_controller->face_angle(-45_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(12 +1, 24), { min_speed: 50 });
    drive_controller->boomerang(pos(18, 18), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_point(lib15442c::Vec(0, 0), 180_deg, { threshold: 10_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 300);

    // ---------- END ---------- //

    // odometry->initialize(30.5 + 48, 31 + 48, 0_deg);
    // intake->set_state(IntakeState::HOOD);
    // drive_controller->face_angle(45_deg);

    // go through ring and get 3rd goal
    drive_controller->boomerang(pos(72 + 24, 72 + 24), { threshold: 9, min_speed: 80 });
    auto face_third_goal = drive_controller->face_angle(135_deg, { threshold: 10_deg, chained: true, async: true });
    pros::delay(200);
    intake->set_state(IntakeState::DISABLED);
    face_third_goal->await();
    drive_controller->boomerang(pose(72, 144 - 24, 105_deg), { backwards: true, lead: 0.4, threshold: 8, min_speed: 60 });
    drive_controller->drive_time(-60, 50);
    clamp.extend();
    drive_controller->drive_time(-60, 50);
    // drive_controller->face_angle(90_deg);
    // pros::delay(100);
    // odometry->set_y(144 - (distance_right.get() * mm_to_in + 6) - 3);
    intake->set_state(IntakeState::HOOD);

    // get rings for 3rd goal and alliance stake
    drive_controller->boomerang(pos(72 - 24, 72 + 24), { threshold: 5, min_speed: 80 });
    drive_controller->face_angle(-45_deg, { threshold: 20_deg, chained: true });
    drive_controller->boomerang(pos(24, 144 - 24), { threshold: 5 });
    pros::delay(300);
    drive_controller->boomerang(pos(24, 144 - 12), { threshold: 10 });
    intake->set_state(IntakeState::WALL_STAKE);
    pros::delay(400);
    drive_controller->drive_time(100, 300);

    // dropoff goal then alliance stake
    drive_controller->boomerang(pos(48, 144 - 30), { backwards: true, threshold: 6, min_speed: 80 });
    drive_controller->boomerang(pos(72 + 10, 144 - 30), { backwards: true, min_speed: 80 });
    clamp.retract();
    pros::delay(50);
    arm->set_state(ArmState::ALLIANCE_STAKE);
    // drive_controller->boomerang(pose(72, 144 - 14, -30_deg), { lead: 0.4, max_speed: 60, min_speed: 35 });
    // drive_controller->face_angle(0_deg, { max_speed: 40 });
    drive_controller->drive(4, { min_speed: 60, chained: true });
    drive_controller->face_point(lib15442c::Vec(72, 144), 0_deg, { threshold: 5_deg, chained: true });
    drive_controller->drive_to(pose(72, 144 - 14, 0_deg), { min_speed: 30 });
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-4, { min_speed: 60, chained: true });

    // push in corner goal
    intake->set_state(IntakeState::DISABLED);
    drive_controller->face_angle(-30_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(72 - 24, 144 - 12), { threshold: 8, min_speed: 127 });
    drive_controller->drive_time(127, 1000);

    // pickup goal and rings
    intake->set_state(IntakeState::REVERSE);
    drive_controller->boomerang(pos(72 + 15, 144 - 30), { backwards: true, threshold: 2, min_speed: 60 });
    intake->set_state(IntakeState::HOOD);
    clamp.extend();
    drive_controller->drive_time(-60, 50);
    pros::delay(50);
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 24, 144 - 24 -4), { lead: 0.4, threshold: 8, min_speed: 60 });
    drive_controller->face_point(lib15442c::Vec(144 - 24, 144 - 12), 0_deg, { threshold: 3_deg });
    drive_controller->drive_time(60, 800);
    drive_controller->face_angle(-45_deg, { threshold: 20_deg, chained: true });
    drive_controller->drive_time(80, 200);

    // drop goal and corner
    drive_controller->boomerang(pos(144 - 20, 144 - 20), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_angle(-135_deg, { threshold: 3_deg, chained: true });
    pros::delay(200);
    clamp.retract();
    drive_controller->drive_time(-127, 500);
    intake->set_state(IntakeState::DISABLED);

    // climb
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->boomerang(pos(144 - 48, 144 - 48), { threshold: 6, min_speed: 90 });
    drive_controller->drive_time(110, 600, { angle: -135_deg });
    pros::delay(100);
    arm->move_manual(-127);
    pros::delay(300);
    arm->move_manual(0);
}