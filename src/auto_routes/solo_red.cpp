#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::solo_red)
{
    RUN_AUTO(red_rush_segment);

    // place ring then drop goal
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_angle(0_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(72 + 30, 24), { backwards: true });
    clamp.retract();
    pros::delay(100);
    
    // get rings out of the way and alliance stake
    drive_controller->boomerang(pos(72, 24), { threshold: 6 });
    pros::delay(500);
    drive_controller->drive(6, { chained: true });
    intake->set_state(IntakeState::WALL_STAKE);
    pros::delay(400);
    drive_controller->drive(-10, { min_speed: 30, chained: true });
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive_to(pose(72 +3, 13, 180_deg), { min_speed: 23 });
    pros::delay(50);
    arm->set_state(ArmState::LOAD);
    pros::delay(100);
    drive_controller->drive(-4, { min_speed: 80, chained: true });

    // get next goal and ring
    drive_controller->boomerang(pos(48, 48), { backwards: true, min_speed: 60 });
    clamp.extend();
    drive_controller->face_angle(-90_deg, { threshold: 20_deg });
    drive_controller->boomerang(pos(24, 48), { threshold: 4 });

    // touch ladder
    arm->set_state(ArmState::LADDER_TOUCH);
    drive_controller->boomerang(pos(48, 48), { backwards: true, threshold: 6 });
    drive_controller->face_angle(45_deg, { threshold: 5_deg });
    drive_controller->drive_time(60, 500);
    intake->set_state(IntakeState::DISABLED);
}