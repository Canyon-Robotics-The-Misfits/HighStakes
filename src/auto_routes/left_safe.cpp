#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::left_safe)
{
    
    pros::Distance distance = pros::Distance(16);

    drivetrain->move(60, 0);
        std::cout << distance.get() << std::endl;
    while (distance.get() > 1000)
    {
        std::cout << distance.get() << std::endl;
        pros::delay(20);
    }
    drivetrain->move(60, 0);
}