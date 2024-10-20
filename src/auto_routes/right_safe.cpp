#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::right_safe)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(144-35, 10.5 + 10)); // TODO: update start position
    
    // pickup goal
    clamp.retract();
    drive_controller->drive_time(-100, 100);
    drive_controller->boomerang(pose(144 - 48, 48, -25_deg), { backwards: true, lead: 0.8, threshold: 8, angle_priority_threshold: 12 });
    drivetrain->move(0,0);
    drive_controller->drive_time(-60, 100);
    clamp.extend();
    drive_controller->drive_time(-60, 50);
    
    // put on rings
    auto ring_pickup = drive_controller->boomerang(pose(144 - 24, 48, 90_deg), { lead: 0.4, max_speed: 60, async: true });
    pros::delay(300);
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    ring_pickup->await();

    // touch ladder
    drive_controller->facePoint(pos(144 - 36, 36).vec(), 180_deg, { chained: true });
    drive_controller->boomerang(pos(144 - 36, 36), { backwards: true, max_speed: 60 });
    drive_controller->faceAngle(-45_deg + 180_deg);
    drive_controller->boomerang(pose(144 - (48 + 12 - 3), 48 + 12 - 3, -45_deg), { backwards: true, threshold: 0.5, angle_priority_threshold: 12, max_speed: 40 });
}