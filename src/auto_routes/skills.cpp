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

    odometry->initialize(57+3, 17.5, 126_deg);

    // score alliance stake
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->drive(-6, { min_speed: 40, chained: true });
    clamp.extend();
    pros::delay(100);
    drive_controller->drive_to(pose(69, 12.5, 164_deg), { r: 4, max_speed: 80 });
    pros::delay(100);
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-7, { min_speed: 40, chained: true });

    // pickup rings
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_point(lib15442c::Vec(48, 48), 0_deg, { threshold: 20_deg, chained: true });
    drive_controller->boomerang(pos(48, 48));

    drive_controller->face_point(lib15442c::Vec(24, 72 + 24), 0_deg, { threshold: 20_deg, chained: true });
    auto redirect_pickup = drive_controller->boomerang(pose(24, 72 + 24 + 5, -5_deg), { async: true });
    pros::delay(200);
    // intake->set_state(IntakeState::WALL_STAKE);
    redirect_pickup->await();
    pros::delay(200);

    // score neutral stake
    drive_controller->face_point(lib15442c::Vec(30, 72 - 4.5), 0_deg, { threshold: 20_deg, chained: true });
    drive_controller->boomerang(pos(30, 72 - 4.5));
    drive_controller->face_angle(-90_deg);
    drive_controller->face_angle(-90_deg, { max_speed: 30 });
    // arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(300);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive(odometry->get_x() - 9);
    drive_controller->drive(-1.5, { min_speed: 30 });
    intake->set_state(IntakeState::DISABLED);
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-7, { min_speed: 80, chained: true });
    intake->set_state(IntakeState::HOOD);

    // get rings near corner
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->boomerang(pos(24+4, 48));
    drive_controller->boomerang(pos(24+4, 12 - 5), { threshold: 4, max_speed: 40 });
    pros::delay(100);
    intake->set_state(IntakeState::REVERSE);
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    pros::delay(50);
    // drive_controller->boomerang(pos(36, 30), { backwards: true });
    // drive_controller->boomerang(pose(12+3, 24, -90_deg));

    // put in corner
    drive_controller->boomerang(pos(24, 24), { backwards: true });
    drive_controller->face_angle(45_deg);
    clamp.retract();
    drive_controller->drive_time(-80, 600);

    // get next goal
    arm->set_state(ArmState::LOAD);
    drive_controller->drive(12, { min_speed: 30, chained: true });
    drive_controller->face_point(lib15442c::Vec(72+24, 24), 180_deg);
    intake->set_state(IntakeState::REVERSE);
    drive_controller->boomerang(pos(72+24 - 24, 24 + 3), { backwards: true, min_speed: 60, chained: true });
    drive_controller->drive_time(-60, 1200);
    clamp.extend();
    pros::delay(100);

    // get more rings
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_point(lib15442c::Vec(144-48, 48), 0_deg, { threshold: 20_deg, chained: true });
    drive_controller->boomerang(pos(144-48, 48));

    drive_controller->face_point(lib15442c::Vec(144-24, 72 + 24), 180_deg, { threshold: 20_deg, chained: true });
    auto redirect_pickup_2 = drive_controller->boomerang(pose(144-24, 72 + 24 + 5, 5_deg), { async: true });
    pros::delay(200);
    // intake->set_state(IntakeState::WALL_STAKE);
    redirect_pickup_2->await();
    pros::delay(200);

    // score neutral stake
    drive_controller->face_point(lib15442c::Vec(144-30, 72 - 4.5), 0_deg, { threshold: 20_deg, chained: true });
    drive_controller->boomerang(pos(144-30, 72 - 4.5));
    drive_controller->face_angle(90_deg);
    drive_controller->face_angle(90_deg, { max_speed: 30 });
    // arm->set_state(ArmState::NEUTRAL_STAKE);
    pros::delay(300);
    intake->set_state(IntakeState::HOOD);
    drive_controller->drive((144 - odometry->get_x()) - 9);
    drive_controller->drive(-1.5, { min_speed: 30 });
    intake->set_state(IntakeState::DISABLED);
    arm->set_state(ArmState::LOAD);
    pros::delay(200);
    drive_controller->drive(-7, { min_speed: 80, chained: true });
    intake->set_state(IntakeState::HOOD);

    // get rings near corner
    arm->set_state(ArmState::ALLIANCE_STAKE);
    drive_controller->boomerang(pos(144 - 24-4, 48));
    drive_controller->boomerang(pos(144 - 24-4, 12 - 5), { threshold: 4, max_speed: 40 });
    pros::delay(100);
    intake->set_state(IntakeState::REVERSE);
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    pros::delay(50);
    // drive_controller->boomerang(pos(144 - 36, 30), { backwards: true });
    // drive_controller->boomerang(pose(144 - 12-3, 24, 90_deg));

    // put in corner
    drive_controller->boomerang(pos(144 - 24, 24), { backwards: true });
    drive_controller->face_angle(-45_deg);
    clamp.retract();
    drive_controller->drive_time(-80, 600);

    // get third goal
    arm->set_state(ArmState::LOAD);
    drive_controller->boomerang(pos(144 - 24, 72 + 24 - 8));
    drive_controller->boomerang(pos(72 + 24, 144 - 24 - 24), { backwards: true, min_speed: 60, chained: true });
    drive_controller->face_angle(135_deg);
    drive_controller->drive_time(-60, 1000);
    clamp.extend();
    pros::delay(100);

    // get more rings
    drive_controller->boomerang(pos(72 + 24, 72 + 24));
    drive_controller->boomerang(pos(72, 72));
    drive_controller->boomerang(pos(72 - 24, 72 + 24));
    drive_controller->boomerang(pos(24, 144 - 24));

    // put in corner
    drive_controller->face_angle(135_deg);
    clamp.retract();
    drive_controller->drive_time(-80, 600);

    // push in other
    drive_controller->boomerang(pos(72, 144-24));
    drive_controller->boomerang(pos(72 + 24, 144-12));
    drive_controller->boomerang(pos(144 - 12, 144 - 12));

    // climb
    arm->set_state(ArmState::NEUTRAL_STAKE);
    drive_controller->boomerang(pos(72 + 24, 72 + 24));
    drive_controller->face_angle(-135_deg);
    drive_controller->drive_time(127, 600);
    arm->move_manual(-127);
    pros::delay(500);
    arm->move_manual(0);
}