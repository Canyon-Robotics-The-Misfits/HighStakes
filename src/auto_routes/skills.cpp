#include "main.h"
#include "autonomous.h"
#include "mechanism/distance_reset.hpp"

#define LOGGER "skills.cpp"

void descore_macro(lib15442c::Pneumatic descore, std::shared_ptr<lib15442c::Pneumatic> lb_lift_push, std::shared_ptr<mechanism::RingManager> rm, std::shared_ptr<mechanism::Arm> lb , std::shared_ptr<lib15442c::Face> turn)
{
    descore.extend();
    rm->idle();
    pros::delay(500);

    rm->intake();
    pros::delay(100);
    rm->intake_reverse();

    pros::delay(300);

    turn->await();

    rm->intake_hold();
    lb_lift_push->extend();
    rm->set_lb_override(true);
    lb->move(-60);
    pros::delay(350);
    lb->move(127);
    pros::delay(150);
    lb->move(0);
    rm->set_lb_override(false);
    lb_lift_push->retract();

    descore.retract();
    rm->idle();
}

AUTO_ROUTE(auto_routes::skills)
{
    odometry->initialize(80, 11, 224.5_deg);
    rm->set_color_sort(mechanism::SortColor::BLUE);

    lb_lift_push->extend();
    rm->set_lb_override(true);
    // lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_SCORE_SKILLS_ANGLE_DEG));
    lb->set_target(105_deg);
    pros::delay(600);
    drive_controller->drive(-23, { max_speed: 80, min_speed: 70, chained: true });
    clamp.extend();
    rm->set_lb_override(false);
    lb_lift_push->retract();

    rm->intake_reverse();
    pros::delay(50);
    rm->stop_intake();
    pros::delay(50);
    rm->load();
    drive_controller->boomerang(pose(144 - 48, 48 -3, 15_deg), { threshold: 5, min_speed: 40 });

    auto drive_to_wall_stake = drive_controller->boomerang(pose(144 -4 - 10 +2, 72 - 3 -1, 80_deg), { lead: 0.5, threshold: 3, max_speed: 80, min_speed: 40, async: true });
    pros::delay(900);
    rm->set_lb_override(true);
    rm->stop_intake();
    lb->set_target(-50_deg);
    pros::delay(150);
    rm->intake_override();
    drive_to_wall_stake->await();

    drive_controller->face_angle(90_deg, { threshold: 5_deg, chained: true });
    drivetrain->move(80, 0);
    pros::delay(25);
    // rm->intake_hold();
    pros::delay(225);
    drivetrain->move(100, 0);
    pros::delay(200);
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_SCORE_SKILLS_ANGLE_DEG));
    pros::delay(350);
    drivetrain->move(0, 0);

    // corner rings, drop goal
    rm->stop_intake();
    drive_controller->drive(-7, { min_speed: 60, chained: true });
    lb->set_target(-35_deg);
    drive_controller->face_angle(180_deg, { threshold: 10_deg, min_speed: 60, chained: true });
    rm->intake();
    auto drive_after_wall_stake_one = drive_controller->boomerang(pose(144 - 24 +1.5, 24 +2, 180_deg), { lead: 0.5, threshold: 2, angle_priority_threshold: 10,  max_speed: 80, min_speed: 30, async: true });
    rm->intake_reverse();
    pros::delay(100);
    rm->intake();
    drive_after_wall_stake_one->await();
    pros::delay(50);
    mechanism::distance_reset(odometry, false, true, false);
    drive_controller->drive(10, { max_speed: 70, min_speed: 40, chained: true });
    auto turn_before_corner_one = drive_controller->face_angle(50_deg, { min_speed: 50, chained: true /*, async: true*/ });
    // pros::delay(200);
    // rm->stop_intake();
    // turn_before_corner_one->await();
    // rm->intake();
    drive_controller->boomerang(pos(144 - 12, 24), { threshold: 4, min_speed: 40 });
    drive_controller->face_angle(-10_deg, { min_speed: 60, chained: true });
    drive_controller->drive(-6, { timeout: 1000, min_speed: 50, chained: true });
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_IDLE_ANGLE_DEG));
    rm->set_lb_override(false);
    clamp.retract();
    rm->intake_reverse();
    pros::delay(50);

    // get ring + goal
    auto leave_corner_one = drive_controller->boomerang(pos(144 - 24 +3, 144 - 48), { threshold: 6, min_speed: 20, async: true });
    pros::delay(400);
    rm->intake();
    // WAIT_UNTIL(odometry->get_position().distance_to(lib15442c::Vec(144 - 24, 144 - 48)) < 3);
    // rm->stop_intake();
    leave_corner_one->await();

    drive_controller->face_angle(0_deg, { min_speed: 20, chained: true });
    rm->stop_intake();
    pros::delay(50);
    mechanism::distance_reset(odometry, true, false, true);
    drive_controller->face_point(lib15442c::Vec(144 - 48, 144 - 12), 180_deg -20_deg, { min_speed: 60, chained: true });
    auto descore_goal_pickup_one = drive_controller->boomerang(pose(144 - 48, 144 - 12 +1, -30_deg + 180_deg), { backwards: true, threshold: 11, angle_priority_threshold: 15, max_speed: 60, min_speed: 40 });
    WAIT_UNTIL(odometry->get_position().distance_to(lib15442c::Vec(144 - 48, 144 - 12)) < 30 || !descore_goal_pickup_one->is_running());
    rm->intake_reverse();
    WAIT_UNTIL(odometry->get_position().distance_to(lib15442c::Vec(144 - 48, 144 - 12)) < 15 || !descore_goal_pickup_one->is_running());
    rm->stop_intake();
    descore_goal_pickup_one->await();
    pros::delay(100);
    clamp.extend();

    // descore
    // pros::delay(100);
    pros::delay(250);
    drive_controller->drive(1, { min_speed: 30, chained: true}); 
    auto descore_turn_one = drive_controller->face_angle(185_deg, { min_speed: 40, chained: true, async: true });
    descore_macro(descore, lb_lift_push, rm, lb, descore_turn_one);

    // ring time
    drive_controller->drive(6, { min_speed: 80, chained: true });
    rm->intake();
    pros::delay(300);
    drive_controller->boomerang(pos(144 - 24, 144 - 24 -2), { threshold: 8, min_speed: 30 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, true, false);
    drive_controller->boomerang(pos(144 - 24 +1, 144 - 12), { threshold: 6 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, false, true);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(144 - 12, 144 - 24), { threshold: 6 });
    // pros::delay(500);
    pros::delay(100);

    // corner two
    drive_controller->face_angle(-145_deg, { threshold: 3_deg, min_speed: 60, chained: true });
    drive_controller->face_angle(-165_deg, { threshold: 3_deg, min_speed: 60, chained: true });
    clamp.retract();
    drive_controller->drive(-8, { timeout: 750, min_speed: 60, chained: true });
    drive_controller->face_angle(-145_deg, { threshold: 3_deg, min_speed: 60, chained: true });
    rm->intake_reverse();
    drive_controller->drive_time(80, 200);

    // goal three and alliance stake
    rm->load();
    drive_controller->boomerang(pos(144 - 48, 144 - 48), { threshold: 2 });
    // drive_controller->boomerang(pose(72, 144 - 24, -30_deg + 180_deg), { backwards: true, threshold: 1, max_speed: 60, min_speed: 30 });
    drive_controller->drive_to(pose(72 +4 -0.5, 144 - 24 -4 -0.5 +1, -30_deg), { r: 6, backwards: true, threshold: 1, max_speed: 60, min_speed: 40 });
    drive_controller->drive(-2, { max_speed: 40, min_speed: 40, chained: true });
    clamp.extend();
    pros::delay(50);

    drive_controller->drive_to(pose(72 +2.5, 144 - 15, 0_deg), { r: 1, threshold: 1, min_speed: 60 });
    // pros::delay(50);
    // mechanism::distance_reset(odometry, true, true, false);
    rm->set_lb_override(true);
    // lb->set_target(-50_deg);
    rm->stop_intake();
    drive_controller->drive_time(40, 600);
    if (abs(odometry->get_rotation().deg()) < 12)
    {
        auto new_pos = lib15442c::Vec(
            72 - 8.5 * sin(odometry->get_rotation().rad()),
            144 - 1 - 8.5 * cos(odometry->get_rotation().rad())
        );

        if (odometry->get_position().distance_to(new_pos) < 6)
        {
            odometry->set_position(new_pos);
        }
        else
        {
            WARN_TEXT("Trig reset >7 inches away from odom positon!");
        }
    }
    else
    {
        WARN_TEXT("Heading out of range for trig reset!");
    }
    drive_controller->drive(-1.5, { threshold: 0, min_speed: 25, chained: true });
    pros::delay(100);
    lb_lift_push->extend();
    pros::delay(25);
    lb->set_target(110_deg);
    pros::delay(500);
    drive_controller->drive(-4, { min_speed: 80, chained: true });

    // run to corner
    auto leave_alliance_stake = drive_controller->boomerang(pose(48 -1, 144 - 48, -127_deg), { threshold: 4, min_speed: 80, async: true });
    pros::delay(200);
    rm->set_lb_override(false);
    pros::delay(600);
    lb_lift_push->retract();
    rm->intake_reverse();
    pros::delay(100);
    rm->intake();
    leave_alliance_stake->await();
    lb_lift_push->retract();
    drive_controller->boomerang(pose(24 +1, 48 +3, 190_deg), { lead: 0.5, threshold: 6, min_speed: 80 });
    drive_controller->boomerang(pose(24 +0.5, 24 +4, 180_deg), { lead: 0.4, threshold: 3, angle_priority_threshold: 6,  max_speed: 90 });
    pros::delay(100);
    mechanism::distance_reset(odometry, true, false, false);
    drive_controller->drive(8, { max_speed: 80, min_speed: 40, chained: true });
    auto turn_before_corner_three = drive_controller->face_angle(-50_deg, { min_speed: 50, chained: true, async: true });
    pros::delay(200);
    rm->stop_intake();
    turn_before_corner_three->await();
    rm->intake();
    drive_controller->boomerang(pos(12, 24 -1), { threshold: 5, min_speed: 40 });
    drive_controller->face_angle(10_deg, { min_speed: 40, chained: true });
    // pros::delay(200);
    drive_controller->drive(-10, { timeout: 500, min_speed: 60, chained: true });
    clamp.retract();
    rm->intake_reverse();
    pros::delay(50);

    // ring + goal 4
    auto leave_corner_four = drive_controller->boomerang(pos(48, 48), { threshold: 3, async: true });
    pros::delay(200);
    rm->load();
    leave_corner_four->await();
    drive_controller->drive_to(pose(48 -4 +0.5, 24 +4 + 0.5, -30_deg + 180_deg), { r: 8, backwards: true, threshold: 1, max_speed: 60, min_speed: 40 });
    drive_controller->drive(-2, { max_speed: 40, min_speed: 40, chained: true });
    rm->stop_load();
    clamp.extend();
    pros::delay(50);

    // wall stake two
    auto drive_to_wall_stake_2 = drive_controller->drive_to(pose(4 +10, 72, -75_deg), { r: 25, threshold: 5, max_speed: 80, min_speed: 40, async: true });
    pros::delay(800);
    rm->stop_intake();
    rm->set_lb_override(true);
    lb->set_target(-45_deg);
    pros::delay(200);
    rm->intake_override();
    drive_to_wall_stake_2->await();

    rm->stop_intake();
    drive_controller->face_angle(-90_deg, { threshold: 3_deg, chained: true });
    drivetrain->move(50, 0);
    pros::delay(300);
    drivetrain->move(70, 0);
    pros::delay(100);
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_SCORE_SKILLS_ANGLE_DEG));
    pros::delay(400);
    drivetrain->move(0, 0);

    // get next ring and drop goal
    rm->intake_override();
    drive_controller->drive(-7, { min_speed: 60, chained: true });
    drive_controller->face_angle(0_deg, { threshold: 10_deg, min_speed: 80, chained: true });
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_IDLE_ANGLE_DEG));
    rm->set_lb_override(false);
    rm->intake();

    drive_controller->boomerang(pos(24 +2, 144 - 48 -4), { threshold: 5 });
    drive_controller->face_angle(0_deg);
    pros::delay(300);
    mechanism::distance_reset(odometry, false, true, true);
    clamp.retract();
    drive_controller->drive(2, { min_speed: 40, chained: true });

    // get next descore goal
    drive_controller->face_point(lib15442c::Vec(48, 144 - 12), 180_deg + 20_deg, { min_speed: 60, chained: true });
    rm->stop_intake();
    drive_controller->boomerang(pose(48, 144 - 12 -1 -2, 30_deg + 180_deg), { backwards: true, threshold: 9, angle_priority_threshold: 15, max_speed: 70, min_speed: 40 });
    pros::delay(100);
    clamp.extend();

    // descore 2 electric boogaloo
    // pros::delay(100);
    pros::delay(250);
    auto descore_turn_two = drive_controller->face_angle(200_deg, { threshold: 5_deg,  min_speed: 40, chained: true, async: true });
    descore_macro(descore, lb_lift_push, rm, lb, descore_turn_two);
    rm->intake_reverse();

    // ring time 2 even more electric boogaloo
    drive_controller->drive(2, { min_speed: 80, chained: true });
    rm->intake();
    drive_controller->boomerang(pos(24, 144 - 24 -6), { threshold: 6, min_speed: 30 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, true, false);
    drive_controller->boomerang(pos(24, 144 - 12), { threshold: 7 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    pros::delay(50);
    mechanism::distance_reset(odometry, false, true, true);
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(12, 144 - 24 +1), { threshold: 5 });
    // pros::delay(500);
    pros::delay(100);

    // corner four electric boogalour
    drive_controller->face_angle(145_deg, { threshold: 3_deg, min_speed: 60, chained: true });
    drive_controller->face_angle(157_deg, { threshold: 3_deg, min_speed: 60, chained: true });
    clamp.retract();
    drive_controller->drive(-8, { timeout: 1000, max_speed: 100, min_speed: 60, chained: true });
    rm->intake_reverse();
    drive_controller->drive_time(80, 200);

    // climb more like 
    rm->intake_high_stake();
    auto grab_high_stake = drive_controller->boomerang(pos(72 -4, 72 -2), { async: true });
    // WAIT_UNTIL(rm->ring_detected() || !grab_high_stake->is_running());
    // rm->intake_reverse();
    // pros::delay(50);
    // rm->stop_intake();
    grab_high_stake->await();
    auto face_climb = drive_controller->face_angle(-90_deg, { async: true });
    rm->prep_climb();
    pros::delay(100);
    rm->intake_hold();
    face_climb->await();
    drive_controller->drive(-10, { min_speed: 50, chained: true});

    rm->climb();
}