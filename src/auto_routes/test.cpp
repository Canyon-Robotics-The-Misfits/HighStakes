#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

AUTO_ROUTE(auto_routes::skills_triple_test)
{
    // RUN_AUTO(skills_start_segment);

    odometry->initialize(83 +1, 17 + 5, 250_deg);

    clamp.retract();

    drive_controller->drive(-4, { min_speed: 60, chained: true });
    clamp.extend();
    pros::delay(100);
    drive_controller->face_point(lib15442c::Vec(144 - 48, 48));

    // get initial rings
    intake->set_state(IntakeState::HOOD);
    drive_controller->boomerang(pos(144 - 48, 48), { threshold: 3, min_speed: 60 });
    drive_controller->drive_time(60, 100);
    drive_controller->boomerang(pos(144 - 12, 72 +2));
    drive_controller->drive(-4, { min_speed: 80, chained: true });
    
    // get rings near corner; dropoff goal
    drive_controller->face_point(lib15442c::Vec(144 - 24, 72 - 24), 0_deg, { threshold: 5_deg, chained: true });
    drive_controller->boomerang(pos(144 - 24, 72 - 24), { threshold: 5, min_speed: 110 });
    drive_controller->boomerang(pos(144 - 24, 24), { threshold: 5, min_speed: 60 });
    drive_controller->boomerang(pos(144 - 24, 12 + 1), { min_speed: 25 });
    pros::delay(150);
    // odometry->set_x(142 - distance_left()); // reset odom w/ distance sensors
    drive_controller->face_angle(45_deg, { threshold: 10_deg, chained: true });
    drive_controller->boomerang(pos(144 - 12 -1, 24), { min_speed: 50 });
    drive_controller->boomerang(pos(144 - 18, 18), { backwards: true, threshold: 5, min_speed: 80 });
    drive_controller->face_point(lib15442c::Vec(144, 0), 180_deg, { threshold: 10_deg, chained: true });
    clamp.retract();
    drive_controller->drive_time(-127, 300);
}

AUTO_ROUTE(auto_routes::corner_clear)
{
    odometry->initialize(0, 0, 0_deg);


    arm->set_state(ArmState::ALLIANCE_STAKE);
    clamp.extend();
    intake->set_state(IntakeState::HOOD);
    pros::delay(500);
    drive_controller->drive_time(50, 500);
    pros::delay(200);
    drive_controller->drive_time(-127, 150);
    drive_controller->drive(-15, { threshold: 0, min_speed: 30, chained: true  });
    pros::delay(100);
    drive_controller->drive_time(60, 300);
    // drive_controller->drive(-15, { threshold: 0, min_speed: 35, chained: true  });
}

AUTO_ROUTE(auto_routes::mp_test)
{
    odometry->initialize(0, 0, 0_deg);

    while (true) {
        std::cout << odometry->get_x() << ", " << odometry->get_y() << std::endl;
        // std::cout << pros::millis() / 1000.0 << ", ";
        drivetrain->move_speed(50.0, -50.0 * (1.0/12.0));
        pros::delay(20);
    }
}