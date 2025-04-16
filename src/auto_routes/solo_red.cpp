#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::solo_red)
{
    odometry->initialize(0, 0, 0_deg);

}