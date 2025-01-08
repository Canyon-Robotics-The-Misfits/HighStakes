#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::positive_red)
{
    odometry->initialize(144-2, 24, -20_deg); // start position is way off the actual one but it works for some reason

    // goal rush
    doinker.extend();
    intake->set_state(IntakeState::HOOD);
    // drive_controller->drive_to(pose(144-23, 53.5, -25_deg), { threshold: 1 });
    drive_controller->drive(37.5, { angle: odometry->get_rotation() - 5_deg, threshold: 1 });
    doinker.retract();
    pros::delay(50);
    drive_controller->drive_time(-127, 250);
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive_time(-127, 150);
    doinker.extend();
    pros::delay(100);
    drive_controller->drive_time(40, 50);
    drive_controller->drive(-6, { threshold: 1, min_speed: 40, chained: true });
    doinker.retract();

    // get next goal
    drive_controller->face_point(lib15442c::Vec(144-48, 48), 180_deg + 20_deg, { min_speed: 40, chained: true });
    drive_controller->drive(-3, { min_speed: 40, chained: true });
    drive_controller->boomerang(pose(144 - 48, 48-5, 90_deg), { backwards: true, lead: 0.6, threshold: 7, min_speed: 60 });
    clamp.extend();
    pros::delay(100);

    // score alliance stake
    // auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+10, 15.5, -145_deg), { min_speed: 25, async: true });
    auto drive_to_alliance_stake = drive_controller->drive_to(pose(72+9, 16.5, -150_deg), { min_speed: 25, async: true });
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    pros::delay(200);
    intake->set_state(IntakeState::HOOD);
    pros::delay(1200);
    intake->set_state(IntakeState::DISABLED);
    drive_to_alliance_stake->await();
    // drive_controller->drive(4, { timeout: 250 });
    arm->set_state(ArmState::LOAD);
    pros::delay(250);
    drive_controller->drive_time(-100, 200);

    // dropoff goal
    drive_controller->face_angle(10_deg, {threshold: 10_deg, chained: true });
    clamp.retract();

    // get rush goal
    drive_controller->drive(15, {min_speed: 60, chained: true });
    drive_controller->face_angle(60_deg + 180_deg, {threshold: 3_deg, chained: true });
    drive_controller->boomerang(pos(144-20, 65), { backwards: true, threshold: 6, min_speed: 60 });
    clamp.extend();

    // get corner 
    arm->set_state(ArmState::ALLIANCE_STAKE);
    intake->set_state(IntakeState::REVERSE);
    drive_controller->boomerang(pose(144-24, 24, -142_deg), { lead: 0.4, threshold: 1.5, min_speed: 25 } );
    drive_controller->drive_time(-100, 150);
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_angle(135_deg, { chained: true });
    drive_controller->face_point(lib15442c::Vec(144, 0));
    // drive_controller->drive_time(80, 250);
    // drive_controller->drive_time(60, 350);
    drive_controller->drive_time(127, 800);
    pros::delay(150);
    drive_controller->drive(-5, {min_speed: 40, chained: true });
    drive_controller->drive_time(100, 100);
    pros::delay(100);
    arm->set_state(ArmState::LADDER_TOUCH);

    // touch ladder
    drive_controller->boomerang(pos(144-36, 36), { backwards: true, threshold: 2, min_speed: 60 } );
    drive_controller->face_angle(-45_deg, { threshold: 5_deg });
    drive_controller->drive_time(80, 500);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

}