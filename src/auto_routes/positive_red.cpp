#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::positive_red)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(144-35, 18.5 + 10));
    
    // rush goal
    clamp.retract();
    drive_controller->drive_time(-127, 200);
    // drive_controller->boomerang(pos(144 - 48, 48), { backwards: true });
    drive_controller->boomerang(pose(144 - 24, 72, 40_deg), { backwards: true, lead: 0.6, threshold: 24, max_speed: 100, min_speed: 40 });
    return;
    drive_controller->drive_time(-60, 300);
    clamp.extend();
    pros::delay(100);
    drive_controller->drive_time(100, 100);
    ring_mech->set_state(mechanism::INTAKE_HOOD);

    // backup and dropoff goal
    drive_controller->drive_to(pose(144 - 35, 24, 180_deg));
    drive_controller->faceAngle(0_deg);
    ring_mech->set_state(mechanism::DISABLED);
    clamp.retract();
    drive_controller->drive_time(100, 250);

    drive_controller->facePoint(lib15442c::Vec(144 - 48, 48), 0_deg, { min_speed: 60, chained: true });
    drive_controller->drive_time(100, 400);
    clamp.extend();
    drive_controller->drive_time(100, 200);
    
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->drive_to(pose(144 - 24, 48, 90_deg));
}