#include "main.h"
#include "autonomous.h"

AUTO_ROUTE_PARAM(auto_routes::negative_blue, bool elims)
{
    odometry->initialize(144 - 42, 19, 16_deg);

    // rush
    doinker.extend();
    auto ring_rush = drive_controller->drive(42, { min_speed: 40, chained: true, async: true });
    rm->intake_reverse();
    pros::delay(200);
    rm->intake();
    pros::delay(300);
    ring_rush->await();
    pros::delay(50);

    // goal grab
    rm->intake_hold();
    drive_controller->drive(-1, { min_speed: 40, chained: true });
    drive_controller->boomerang(pos(144 - 48, 48 +1), { backwards: true, threshold: 3, min_speed: 35 });
    clamp.extend();
    rm->intake();
    pros::delay(50);

    // next rings
    drive_controller->face_angle(50_deg, { min_speed: 40, chained: true });
    doinker.retract();
    pros::delay(250);
    drive_controller->boomerang(pos(144 - 24, 48 -1), { threshold: 3, max_speed: 70, min_speed: 40 });

    // corner
    drive_controller->boomerang(pose(144 - 10, 10, 135_deg), { lead: 0.4, threshold: 4, min_speed: 40 });
    drive_controller->drive_time(60, 300);
    pros::delay(200);
    intake_lift.extend();
    drive_controller->drive(-2.5, { min_speed: 50, chained: true });
    pros::delay(50);
    drive_controller->drive(4, { timeout: 500, min_speed: 60, chained: true });
    pros::delay(100);
    intake_lift.retract();
    drive_controller->drive(-10, { min_speed: 80, chained: true });

    // next rings 2
    auto pickup_preload = drive_controller->boomerang(pos(144 - 55, 24 - 5), { min_speed: 30, async: true });
    pros::delay(700);
    rm->load();
    pickup_preload->await();
    pros::delay(200);
    intake_lift.extend();
    pros::delay(100);
    rm->set_lb_override(true);
    lb->set_target(-20_deg);
    drive_controller->boomerang(pose(72, 24 -1, -50_deg), { lead: 0.3, threshold: 4, max_speed: 60 });
    pros::delay(300);
    intake_lift.retract();

    // alliance stake
    drivetrain->move(0, 80);
    pros::delay(700);
    drive_controller->face_angle(180_deg, { max_speed: 80, min_speed: 40, chained: true });
    rm->stop_intake();
    drive_controller->boomerang(pos(72, 4), { threshold: 10 });
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
    drive_controller->drive(11, { timeout: 750, min_speed: 50, chained: true });
}