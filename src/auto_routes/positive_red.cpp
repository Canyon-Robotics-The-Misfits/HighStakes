#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;

AUTO_ROUTE(auto_routes::positive_red)
{
    odometry->initialize(144-10, 22, -20_deg);

    // goal rush
    doinker.extend();
    intake->set_state(IntakeState::HOOD);
    // drive_controller->drive_to(pose(144-23, 53.5, -25_deg), { threshold: 1 });
    drive_controller->drive(37.5, { threshold: 1 });
    intake->set_state(IntakeState::DISABLED);
    doinker.retract();
    pros::delay(50);
    drive_controller->drive_time(-127, 400);
    doinker.extend();
    drive_controller->drive_time(40, 100);
    drive_controller->drive(-10, { threshold: 1 });
    doinker.retract();

    // get next goal
    // drive_controller->face_angle(-60_deg, { min_speed: 60, chained: true });
    drive_controller->boomerang(pos(144 - 36, 36), { threshold: 2 });
    std::cout << odometry->get_x() << ", " << odometry->get_y() << std::endl;
    drive_controller->face_angle(135_deg);
    drive_controller->drive(-20);
    clamp.extend();
}