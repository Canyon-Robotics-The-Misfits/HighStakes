#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::solo_blue)
{
    odometry->initialize(0, 0, 0_deg);

}