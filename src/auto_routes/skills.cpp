#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::skills)
{
    odometry->initialize(144 - 63, 9, 226.5_deg);

    drive_controller->boomerang(pos(144 - 48, 48), { backwards: true });
    pros::delay(1000);
    std::cout << odometry->get_x() << ", " << odometry->get_y() << std::endl;
    drive_controller->face_angle(0_deg);
    pros::delay(1000);
    std::cout << odometry->get_x() << ", " << odometry->get_y() << std::endl;
    return;

    // rm->set_lb_override(true);
    // lb->set_target(50_deg);
    // WAIT_UNTIL(lb->is_settled());
    drive_controller->drive(-18, { min_speed: 60, chained: true });
    clamp.extend();
    rm->set_lb_override(false);

    rm->load();
    // drive_controller->drive_to(pose(144 - 48, 48, 45_deg), { threshold: 5, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 48, 48), { threshold: 5, min_speed: 60 });
    drive_controller->drive_to(pose(144 - 12, 72, 0_deg), { threshold: 5, min_speed: 60 });
    rm->set_lb_override(true);
    lb->set_target(34_deg);
    rm->intake();
    WAIT_UNTIL(lb->is_settled());
    rm->set_lb_override(false);
}