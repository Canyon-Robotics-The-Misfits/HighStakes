#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"


// AUTO_ROUTE(auto_routes::mp_test)
// {
//     odometry->initialize(0, 0, 0_deg);

//     lib15442c::TrajectoryBuilder trajectory_builder = lib15442c::TrajectoryBuilder({ point: lib15442c::Vec(0, 0), tangent: lib15442c::Vec(0, 50) });
//     trajectory_builder.append_hermite({ point: lib15442c::Vec(0, 48), tangent: lib15442c::Vec(0, 50) });

//     auto trajectory = trajectory_builder.compute(config::TRAJECTORY_CONSTRAINTS, -1, true);
//     // trajectory.debug_log();

//     lib15442c::RAMSETE ramsete = lib15442c::RAMSETE(trajectory, 0.013, 0.7);

//     ramsete.execute(drivetrain, odometry);

//     // while (true) {
//     //     std::cout << odometry->get_x() << ", " << odometry->get_y() << std::endl;
//     //     // std::cout << pros::millis() / 1000.0 << ", ";
//     //     drivetrain->move_speed(50.0, -50.0 * (1.0/12.0));
//     //     pros::delay(20);
//     // }
// }