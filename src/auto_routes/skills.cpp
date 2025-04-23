#include "main.h"
#include "autonomous.h"
#include "mechanism/distance_reset.hpp"

AUTO_ROUTE(auto_routes::skills)
{
    odometry->initialize(80, 11, 224.5_deg);
    rm->set_color_sort(mechanism::SortColor::NONE);

    // lb_lift_push->extend();
    rm->set_lb_override(true);
    lb->move(127);
    pros::delay(300);
    lb->move(60);
    pros::delay(100);
    lb->move(0);
    pros::delay(100);
    drive_controller->drive(-23, { max_speed: 80, min_speed: 70, chained: true });
    clamp.extend();
    rm->set_lb_override(false);
    lb_lift_push->retract();

    pros::delay(100);
    rm->load();
    drive_controller->boomerang(pose(144 - 48, 48, 30_deg), { threshold: 5, min_speed: 40 });

    // drive_controller->boomerang(pos(144 - 11 - 12, 72 - 7), { threshold: 20, min_speed: 60 });
    // auto drive_to_wall_stake = drive_controller->drive_to(pose(144 - 11, 72, 85_deg), { r: 2, threshold: 5, min_speed: 20, async: true });
    auto drive_to_wall_stake = drive_controller->boomerang(pose(144 -4 - 10, 72 - 3, 80_deg), { lead: 0.5, threshold: 3, max_speed: 80, min_speed: 40, async: true });
    pros::delay(800);
    rm->stop_intake();
    rm->set_lb_override(true);
    lb->set_target(-30_deg);
    drive_to_wall_stake->await();

    rm->intake_override();
    drive_controller->face_angle(90_deg, { threshold: 5_deg, chained: true });
    drivetrain->move(50, 0);
    pros::delay(400);
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_SCORE_SKILLS_ANGLE_DEG));
    pros::delay(300);
    drivetrain->move(0, 0);
    rm->set_lb_override(false);

    // corner rings, drop goal
    drive_controller->drive(-8, { min_speed: 40, chained: true });
    rm->intake();
    drive_controller->face_angle(180_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pose(144 - 24, 12, 180_deg), { threshold: 3, angle_priority_threshold: 10,  max_speed: 90 });
    pros::delay(600);
    mechanism::distance_reset(odometry, false, true, false);
    drive_controller->face_angle(50_deg, { min_speed: 50, chained: true });
    drive_controller->boomerang(pos(144 - 12, 24), { threshold: 5, min_speed: 40 });
    drive_controller->face_angle(-20_deg, { min_speed: 40, chained: true });
    pros::delay(200);
    drive_controller->drive(-10, { max_speed: 80, min_speed: 60, chained: true });
    clamp.retract();
    rm->intake_reverse();
    pros::delay(50);

    // get ring + goal
    auto leave_corner_one = drive_controller->boomerang(pos(144 - 24, 144 - 48), { threshold: 6, min_speed: 20, async: true });
    pros::delay(100);
    rm->intake();
    leave_corner_one->await();
    auto face_goal_two = drive_controller->face_point(lib15442c::Vec(144 - 48, 144 - 12), 180_deg -20_deg, { min_speed: 40, chained: true, async: true });
    pros::delay(0);
    rm->stop_intake();
    face_goal_two->await();
    drive_controller->boomerang(pose(144 - 48, 144 - 12 +1, -30_deg + 180_deg), { backwards: true, threshold: 12, max_speed: 60, min_speed: 63.5 });
    clamp.extend();

    // descore
    pros::delay(1);
    auto descore_drive = drive_controller->drive(4, { max_speed: 40, min_speed: 30, chained: true, async: true });
    descore.extend();
    rm->intake_reverse();
    rm->set_lb_override(true);
    lb->set_target(-64_deg);
    pros::delay(500);
    rm->stop_intake();
    lb_lift_push->extend();
    pros::delay(200);
    descore.retract();
    pros::delay(200);
    lb->move(127);
    pros::delay(300);
    lb_lift_push->retract();
    rm->set_lb_override(false);
    lb->move(0);
    descore_drive->await();

    // ring time
    pros::delay(300);
    rm->intake();
    drive_controller->drive(6, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(144 - 24, 144 - 24 -3), { threshold: 2, min_speed: 30 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, false, false);
    drive_controller->boomerang(pos(144 - 24, 144 - 12), { threshold: 6 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    drive_controller->face_angle(135_deg, { threshold: 4_deg });
    drive_controller->boomerang(pos(144 - 12, 144 - 24), { threshold: 4 });
    pros::delay(500);

    // corner two
    drive_controller->face_angle(-135_deg, { min_speed: 60, chained: true });
    clamp.retract();
    drive_controller->drive_time(-80, 400);
    drive_controller->drive_time(127, 200);

    // goal three and alliance stake
    rm->intake();
    drive_controller->boomerang(pos(144 - 48, 144 - 48), { threshold: 4 });
    rm->load();
    drive_controller->boomerang(pos(72, 144 - 24), { backwards: true, threshold: 4, min_speed: 40 });
    clamp.extend();
    pros::delay(50);

    drive_controller->drive_to(pose(72, 144 - 12, 0_deg), { r: 2, min_speed: 40 });
    mechanism::distance_reset(odometry, true, true, false);
    rm->set_lb_override(true);
    lb->set_target(-30_deg);
    rm->stop_intake();
    drive_controller->drive_time(40, 400);
    drive_controller->drive(-2, { min_speed: 25, chained: true });
    lb_lift_push->extend();
    lb->set_target(90_deg);
    pros::delay(300);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    lb_lift_push->retract();
    rm->set_lb_override(false);

    // run to corner
    rm->intake();
    drive_controller->boomerang(pos(48, 144 - 48), { threshold: 4, min_speed: 80 });
    drive_controller->boomerang(pose(24, 48, 180_deg), { lead: 0.4, threshold: 6, min_speed: 80 });
    drive_controller->boomerang(pos(24, 12), { threshold: 4, max_speed: 100, min_speed: 80 });
    pros::delay(600);
    mechanism::distance_reset(odometry, true, false, false);
    drive_controller->face_angle(-50_deg, { min_speed: 50, chained: true });
    drive_controller->boomerang(pos(12, 24), { threshold: 5, min_speed: 40 });
    drive_controller->face_angle(20_deg, { min_speed: 40, chained: true });
    pros::delay(200);
    drive_controller->drive(-10, { max_speed: 80, min_speed: 60, chained: true });
    clamp.retract();
    rm->intake_reverse();
    pros::delay(50);

}