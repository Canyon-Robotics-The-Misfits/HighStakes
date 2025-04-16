#include "main.h"
#include "autonomous.h"

AUTO_ROUTE_PARAM(auto_routes::negative_blue, bool elims)
{
    odometry->initialize(0, 0, 0_deg);

}