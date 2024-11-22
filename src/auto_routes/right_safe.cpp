#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::right_safe)
{
    odometry->initialize(144 - 35, 10.5 + 10, 180_deg);
    
    // pickup goal
    clamp.retract();
    drive_controller->drive_time(-100, 100);
    drive_controller->boomerang(pose(144 - 48, 48, -30_deg), { backwards: true, lead: 0.8, threshold: 8, angle_priority_threshold: 10 });
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
    drive_controller->face_point(pos(144 - 36, 36).vec(), 180_deg, { chained: true });
    drive_controller->boomerang(pos(144 - 36, 36), { backwards: true, threshold: 15, max_speed: 60 });
    drive_controller->boomerang(pose(144 - (48 + 12 - 4), 48 + 12 - 4, -45_deg), { backwards: true, lead: 0.8, threshold: 0.5, angle_priority_threshold: 5, max_speed: 40 });
}