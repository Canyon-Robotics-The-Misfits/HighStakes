#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::skills)
{
    
    // drive_controller->faceAngle(90_deg, { timeout: INFINITY });
    // drive_controller->faceAngle(180_deg, { timeout: INFINITY });
    // drive_controller->faceAngle(0_deg, { timeout: INFINITY });
    // drive_controller->faceAngle(10_deg, { timeout: INFINITY });

    drive_controller->drive(24, { timeout: INFINITY });
    pros::delay(1000);
    drive_controller->drive(12, { timeout: INFINITY });
    pros::delay(1000);
    drive_controller->drive(-12, { timeout: INFINITY });
}