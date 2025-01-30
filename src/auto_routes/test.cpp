#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

AUTO_ROUTE(auto_routes::skills_triple_test)
{
    RUN_AUTO(skills_start_segment);

    // get initial rings
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 48, 48), { threshold: 3, min_speed: 60 });
    drive_controller->drive_time(60, 100);
    drive_controller->boomerang(pos(144 - 14, 72 +2));
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    
    // get rings near corner; dropoff goal
    drive_controller->face_point(lib15442c::Vec(144 - 24, 72 - 24), 0_deg, { threshold: 15_deg, chained: true });
    drive_controller->boomerang(pos(144 - 24, 72 - 24), { threshold: 5, min_speed: 110 });
    drive_controller->boomerang(pos(144 - 24, 12 + 1), { min_speed: 25 });
    pros::delay(50);
    drive_controller->face_angle(45_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(144 - 12 -1, 24), { min_speed: 50 });
    drive_controller->boomerang(pos(144 - 18, 18), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_point(lib15442c::Vec(144, 0), 180_deg, { threshold: 10_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 300);
}