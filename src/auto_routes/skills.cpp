#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::skills)
{
    drive_controller->faceAngle(90_deg);
}