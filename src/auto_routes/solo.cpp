#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::solo)
{
    drive_controller->boomerang(pos(24, 72), { max_speed: 80 });
    drive_controller->face_point(pos(2, 72).vec(), 0_deg, { min_speed: 23 });
    ring_mech->set_state(mechanism::ARM_NEUTRAL_STAKE);
    pros::delay(500);
    drive_controller->boomerang(pos(2, 72), { threshold: 12 });
    ring_mech->set_state(mechanism::INTAKE_WALL_STAKE);
    pros::delay(300);
    drive_controller->drive_time(-100, 300);
}