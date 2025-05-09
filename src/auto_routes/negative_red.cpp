#include "main.h"
#include "autonomous.h"

AUTO_ROUTE_PARAM(auto_routes::negative_red, bool elims)
{
    odometry->initialize(42, 19, -16_deg);
    rm->set_color_sort(mechanism::SortColor::BLUE);

    // rush
    lb_lift_push->retract();
    auto ring_rush = drive_controller->drive(44, { angle: -17_deg, min_speed: 40, chained: true, async: true });
    pros::delay(200);
    rm->intake_reverse();
    pros::delay(150);
    rm->intake();
    doinker.extend();
    ring_rush->await();

    // goal grab
    drive_controller->drive(-1, { min_speed: 40, chained: true });
    rm->intake_hold();
    drive_controller->boomerang(pos(48, 48 +1), { backwards: true, threshold: 3, min_speed: 50 });
    rm->intake();
    drive_controller->drive(-1, { min_speed: 60, chained: true });
    clamp.extend();
    pros::delay(50);

    // next rings
    // drive_controller->face_angle(-50_deg, { min_speed: 40, chained: true });
    doinker.retract();
    pros::delay(100);
    drive_controller->boomerang(pose(24, 48 -2, -80_deg), { threshold: 3, max_speed: 60, min_speed: 40 });
    pros::delay(100);
    drive_controller->drive(-2, { min_speed: 40, chained: true });

    // corner
    drive_controller->boomerang(pos(10.5, 11), { threshold: 2, min_speed: 25 });
    drive_controller->drive_time(40, 200);
    // pros::delay(300);
    drive_controller->drive(-2.5, { min_speed: 60, chained: true });
    pros::delay(100);
    // intake_lift.extend();
    // drive_controller->drive(-1, { max_speed: 50, min_speed: 40, chained: true });
    pros::delay(150);
    // drive_controller->drive(5, { timeout: 500, min_speed: 60, chained: true });
    // pros::delay(100);
    // intake_lift.retract();
    drive_controller->drive(-10, { min_speed: 80, chained: true });

    // next rings 2
    auto pickup_preload = drive_controller->boomerang(pos(55, 24 - 4), { min_speed: 30, async: true });
    pros::delay(700);
    rm->load();
    pickup_preload->await();
    pros::delay(200);
    intake_lift.extend();
    pros::delay(250);
    rm->set_lb_override(true);
    lb->set_target(-20_deg);
    rm->intake();
    drive_controller->boomerang(pos(72, 24 +2), { threshold: 2, max_speed: 60 });
    pros::delay(200);
    intake_lift.retract();

    // alliance stake
    // doinker.extend();
    auto doinker_sweep = drive_controller->face_angle(-125_deg, { min_speed: 100, chained: true, async: true });
    pros::delay(450);
    // doinker.retract();
    rm->intake_reverse();
    doinker_sweep->await();
    rm->intake();
    drive_controller->face_angle(180_deg, { max_speed: 80, min_speed: 40, chained: true });
    auto approach_alliance_stake = drive_controller->boomerang(pos(72, 4), { threshold: 10, timeout: 1000, async: true });
    pros::delay(150);
    rm->stop_intake();
    approach_alliance_stake->await();
    drive_controller->drive_time(40, 100);
    lb_lift_push->extend();
    drive_controller->drive(-1.5, { threshold: 0, min_speed: 20, chained: true });
    lb->set_target(110_deg);
    pros::delay(500);

    // touch ladder
    drive_controller->drive(-6, { min_speed: 50, chained: true });
    rm->set_lb_override(false);
    drive_controller->face_point(lib15442c::Vec(72, 48), 0_deg, { min_speed: 60, chained: true });
    lb_lift_push->retract();
    drive_controller->drive(9, { timeout: 750, min_speed: 50, chained: true });
    drive_controller->drive(3, { max_speed: 50, min_speed: 40, chained: true });
}