#include "main.h"
#include "autonomous.h"
#include "mechanism/distance_reset.hpp"

void descore_macro(lib15442c::Pneumatic descore, std::shared_ptr<lib15442c::Pneumatic> lb_lift_push, std::shared_ptr<mechanism::RingManager> rm, std::shared_ptr<mechanism::Arm> lb)
{
    descore.extend();
    rm->intake_reverse();
    rm->set_lb_override(true);
    lb->set_target(-64_deg);
    pros::delay(500);
    rm->stop_intake();
    pros::delay(100);
    rm->intake_reverse();
    pros::delay(300);
    rm->stop_intake();
    lb_lift_push->extend();
    pros::delay(300);
    descore.retract();
    lb->move(127);
    pros::delay(200);
    lb_lift_push->retract();
    pros::delay(100);
    rm->set_lb_override(false);
    rm->idle();
}

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
    drive_controller->boomerang(pose(144 - 48, 48, 20_deg), { threshold: 5, min_speed: 40 });

    auto drive_to_wall_stake = drive_controller->boomerang(pose(144 -4 - 10, 72 - 3, 80_deg), { lead: 0.5, threshold: 3, max_speed: 80, min_speed: 40, async: true });
    pros::delay(700);
    rm->stop_intake();
    rm->set_lb_override(true);
    lb->set_target(-20_deg);
    drive_to_wall_stake->await();

    rm->intake_override();
    drive_controller->face_angle(90_deg, { threshold: 5_deg, chained: true });
    drivetrain->move(50, 0);
    pros::delay(400);
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_SCORE_SKILLS_ANGLE_DEG));
    pros::delay(300);
    drivetrain->move(0, 0);

    // corner rings, drop goal
    drive_controller->drive(-8, { min_speed: 60, chained: true });
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_IDLE_ANGLE_DEG));
    rm->set_lb_override(false);
    rm->intake_reverse();
    drive_controller->face_angle(180_deg, { min_speed: 40, chained: true });
    rm->intake();
    drive_controller->boomerang(pose(144 - 24, 12, 180_deg), { threshold: 3, angle_priority_threshold: 10,  max_speed: 90 });
    pros::delay(600);
    mechanism::distance_reset(odometry, false, true, false);
    drive_controller->face_angle(50_deg, { min_speed: 50, chained: true });
    drive_controller->boomerang(pos(144 - 12, 24), { threshold: 5, min_speed: 40 });
    drive_controller->face_angle(-10_deg, { min_speed: 40, chained: true });
    // pros::delay(200);
    drive_controller->drive(-10, { min_speed: 50, chained: true });
    clamp.retract();
    rm->intake_reverse();
    pros::delay(50);

    // get ring + goal
    auto leave_corner_one = drive_controller->boomerang(pos(144 - 24, 144 - 48), { threshold: 6, min_speed: 20, async: true });
    pros::delay(100);
    rm->intake();
    WAIT_UNTIL(odometry->get_position().distance_to(lib15442c::Vec(144 - 24, 144 - 48)) < 10);
    rm->stop_intake();
    leave_corner_one->await();

    auto face_reset_two = drive_controller->face_angle(0_deg, { min_speed: 20, chained: true, async: true });
    pros::delay(0);
    rm->stop_intake();
    face_reset_two->await();
    pros::delay(50);
    mechanism::distance_reset(odometry, true, false, true);
    drive_controller->face_point(lib15442c::Vec(144 - 48, 144 - 12), 180_deg -20_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pose(144 - 48, 144 - 12 -1, -30_deg + 180_deg), { backwards: true, threshold: 10, angle_priority_threshold: 15, max_speed: 60, min_speed: 40 });
    pros::delay(100);
    clamp.extend();

    // descore
    pros::delay(50);
    drive_controller->drive(2, { max_speed: 40, min_speed: 30, chained: true });
    descore_macro(descore, lb_lift_push, rm, lb);
    rm->intake();

    // ring time
    pros::delay(300);
    rm->intake();
    drive_controller->drive(2, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(144 - 24, 144 - 24 -3), { threshold: 6, min_speed: 30 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, true, false);
    drive_controller->boomerang(pos(144 - 24, 144 - 12), { threshold: 6 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, false, true);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(144 - 12, 144 - 24), { threshold: 7 });
    pros::delay(500);

    // corner two
    drive_controller->face_angle(-165_deg, { min_speed: 60, chained: true });
    clamp.retract();
    drive_controller->drive_time(-80, 400);
    drive_controller->drive_time(80, 200);

    // goal three and alliance stake
    rm->intake();
    drive_controller->boomerang(pos(144 - 48, 144 - 48), { threshold: 4 });
    rm->load();
    drive_controller->boomerang(pos(72, 144 - 24), { backwards: true, threshold: 3, max_speed: 60, min_speed: 30 });
    clamp.extend();
    pros::delay(50);

    drive_controller->drive_to(pose(72 +3, 144 - 15, 0_deg), { r: 2, min_speed: 50 });
    pros::delay(50);
    mechanism::distance_reset(odometry, true, true, false);
    rm->set_lb_override(true);
    lb->set_target(-30_deg);
    rm->stop_intake();
    drive_controller->drive_time(40, 500);
    odometry->set_x(72 + 7.5 * sin(odometry->get_rotation().rad()));
    drive_controller->drive(-3, { min_speed: 25, chained: true });
    lb_lift_push->extend();
    lb->set_target(90_deg);
    pros::delay(300);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    lb_lift_push->retract();
    rm->set_lb_override(false);

    // run to corner
    rm->intake();
    drive_controller->boomerang(pose(48, 144 - 48, -140_deg), { threshold: 4, min_speed: 80 });
    drive_controller->boomerang(pose(24, 48, 180_deg), { lead: 0.4, threshold: 6, min_speed: 80 });
    drive_controller->boomerang(pose(24, 12, 180_deg), { threshold: 3, angle_priority_threshold: 10,  max_speed: 90 });
    pros::delay(600);
    mechanism::distance_reset(odometry, true, false, false);
    drive_controller->face_angle(-50_deg, { min_speed: 50, chained: true });
    drive_controller->boomerang(pos(12, 24), { threshold: 5, min_speed: 40 });
    drive_controller->face_angle(10_deg, { min_speed: 40, chained: true });
    // pros::delay(200);
    drive_controller->drive(-10, { min_speed: 60, chained: true });
    clamp.retract();
    rm->intake_reverse();
    pros::delay(50);

    // ring + goal 4
    rm->load();
    drive_controller->boomerang(pos(48, 48));
    drive_controller->drive_to(pose(48 -4 +0.5, 24 +4 + 0.5, -30_deg + 180_deg), { r: 8, backwards: true, threshold: 1, max_speed: 60, min_speed: 40 });
    drive_controller->drive(-2, { max_speed: 40, min_speed: 40, chained: true });
    clamp.extend();
    pros::delay(50);

    // wall stake two
    auto drive_to_wall_stake_2 = drive_controller->drive_to(pose(4 +10, 72 - 3, -85_deg), { r: 20, threshold: 5, max_speed: 80, min_speed: 40, async: true });
    pros::delay(500);
    rm->stop_intake();
    rm->set_lb_override(true);
    lb->set_target(-20_deg);
    drive_to_wall_stake_2->await();

    rm->intake_override();
    drive_controller->face_angle(-90_deg, { threshold: 3_deg, chained: true });
    drivetrain->move(50, 0);
    pros::delay(400);
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_SCORE_SKILLS_ANGLE_DEG));
    pros::delay(300);
    drivetrain->move(0, 0);
    rm->set_lb_override(false);

    // get next ring and drop goal
    drive_controller->drive(-8, { min_speed: 60, chained: true });
    lb->set_target(lib15442c::Angle::from_deg(mechanism::LB_IDLE_ANGLE_DEG));
    rm->set_lb_override(false);
    rm->intake_reverse();
    drive_controller->face_angle(0_deg, { min_speed: 40, chained: true });
    rm->intake();

    drive_controller->boomerang(pos(24, 144 - 48), { threshold: 4 });
    pros::delay(400);
    mechanism::distance_reset(odometry, false, true, true);
    clamp.retract();
    drive_controller->drive(2, { min_speed: 40, chained: true });

    // get next descore goal
    drive_controller->face_point(lib15442c::Vec(48, 144 - 12), 180_deg + 20_deg, { min_speed: 40, chained: true });
    drive_controller->boomerang(pose(48, 144 - 12 -1, 30_deg + 180_deg), { backwards: true, threshold: 10, angle_priority_threshold: 15, max_speed: 60, min_speed: 40 });
    pros::delay(100);
    clamp.extend();

    // descore 2 electric boogaloo
    pros::delay(50);
    drive_controller->drive(2, { max_speed: 40, min_speed: 30, chained: true });
    descore_macro(descore, lb_lift_push, rm, lb);
    rm->intake();

    // ring time 2 even more electric boogaloo
    drive_controller->drive(2, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(24, 144 - 24 -3), { threshold: 6, min_speed: 30 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, true, true, false);
    drive_controller->boomerang(pos(24, 144 - 12), { threshold: 6 });
    drive_controller->face_angle(0_deg, { threshold: 4_deg });
    mechanism::distance_reset(odometry, false, true, true);
    drive_controller->drive(-6, { min_speed: 80, chained: true });
    drive_controller->boomerang(pos(12, 144 - 24), { threshold: 7 });
    pros::delay(500);

    // corner four electric boogalour
    drive_controller->face_angle(165_deg, { min_speed: 60, chained: true });
    clamp.retract();
    drive_controller->drive_time(-80, 400);
    drive_controller->drive_time(80, 200);

    // climb more like 
    rm->intake();
    drive_controller->boomerang(pos(72, 72), { threshold: 10, min_speed: 40 });
    rm->stop_intake();
    rm->prep_climb();
    drive_controller->face_angle(-90_deg);
    drive_controller->drive(-16, { min_speed: 40, chained: true});
}