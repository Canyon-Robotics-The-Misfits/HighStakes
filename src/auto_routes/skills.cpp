#include "main.h"
#include "autonomous.h"

#include "lib15442c/trajectory/trajectory_builder.hpp"
#include "lib15442c/motion/ramsete.hpp"
#include "config.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::skills)
{
    /*
    odometry->initialize(0, 0, 0_deg);

    lib15442c::TrajectoryBuilder trajectory_builder = lib15442c::TrajectoryBuilder({ point: lib15442c::Vec(0, 0), tangent: lib15442c::Vec(0, 48) });
    trajectory_builder.append_hermite({ point: lib15442c::Vec(24, 48), tangent: lib15442c::Vec(0, 48) });

    auto trajectory = trajectory_builder.compute(config::TRAJECTORY_CONSTRAINTS, -1, true);
    trajectory.debug_log();

    lib15442c::RAMSETE ramsete = lib15442c::RAMSETE(trajectory, 0.0013, 0.7);

    ramsete.execute(drivetrain, odometry);
    */

    odometry->initialize(57, 16, 137_deg);

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->drive(-6, { min_speed: 60, chained: true });
    clamp.extend();
    pros::delay(100);
    drive_controller->drive_to(pose(62.7848, 9.23161, 164_deg), { min_speed: 30 });
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-7, { min_speed: 80, chained: true });

    // pickup rings for neutral stake
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->face_point(lib15442c::Vec(48, 48), 0_deg, { threshold: 20_deg, chained: true });
    drive_controller->boomerang(pos(48, 48), { min_speed: 80 });
    drive_controller->boomerang(pos(24, 48), { min_speed: 80 });
    drive_controller->boomerang(pos(36, 72), { min_speed: 80 });
    drive_controller->face_angle(-90_deg);

    // score alliance stake
    arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(300);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(odometry->get_x() - 13);
    intake->set_state(IntakeState::DISABLED);
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-7, { min_speed: 80, chained: true });
    intake->set_state(IntakeState::HOOD);

    // get mores rings
    drive_controller->face_point(lib15442c::Vec(24, 72 + 24));
    return;
    drive_controller->boomerang(pos(24, 72 + 24), { min_speed: 60 });
    drive_controller->boomerang(pos(24, 144 - 24), { threshold: 6, min_speed: 60 });
    pros::delay(200);
    drive_controller->drive(-10, { min_speed: 60, chained: true });
    intake->set_state(IntakeState::WALL_STAKE);
    drive_controller->boomerang(pos(12, 144 - 24), { threshold: 6, min_speed: 60 });
    drive_controller->drive(8, { max_speed: 80, min_speed: 40, chained: true });

}