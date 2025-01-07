#include "main.h"
#include "autonomous.h"

using mechanism::IntakeState;
using mechanism::ArmState;

AUTO_ROUTE(auto_routes::positive_red)
{
    odometry->initialize(144-2, 22, -20_deg); // start position is way off the actual one but it works here for some reason

    // goal rush
    doinker.extend();
    intake->set_state(IntakeState::HOOD);
    // drive_controller->drive_to(pose(144-23, 53.5, -25_deg), { threshold: 1 });
    drive_controller->drive(37.5, { angle: odometry->get_rotation() - 5_deg, threshold: 1 });
    doinker.retract();
    pros::delay(50);
    drive_controller->drive_time(-127, 300);
    intake->set_state(IntakeState::DISABLED);
    drive_controller->drive_time(-127, 100);
    doinker.extend();
    drive_controller->drive_time(40, 100);
    drive_controller->drive(-10, { threshold: 1, min_speed: 40, chained: true });
    doinker.retract();

    // get next goal
    drive_controller->boomerang(pos(144 - 36, 36), { threshold: 2 });
    // drive_controller->face_angle(odometry->get_pose().vec().angle_to(lib15442c::Vec(144-48, 48)) + 180_deg, { threshold: 5_deg });
    // drive_controller->face_angle(odometry->get_pose().vec().angle_to(lib15442c::Vec(144-48, 48)) + 180_deg);
    drive_controller->face_point(lib15442c::Vec(144-48, 48), 180_deg, { threshold: 5_deg });
    drive_controller->drive(-16, { min_speed: 80, chained: true });
    clamp.extend();
    pros::delay(100);
    intake->set_state(IntakeState::HOOD);
    pros::delay(150);

    // dropoff goal
    drive_controller->face_angle(-10_deg, {threshold: 3_deg, chained: true });
    clamp.retract();
    drive_controller->drive(-15, { min_speed: 40, chained: true });

    // score alliance stake (replace dropoff goal)
    // drive_controller->boomerang(pose(12, 72+12, -135_deg))

    // get rush goal
    drive_controller->drive(3, {min_speed: 60, chained: true });
    drive_controller->face_angle(60_deg + 180_deg, {threshold: 3_deg, chained: true });
    drive_controller->boomerang(pos(144-20, 65), { backwards: true, threshold: 7, min_speed: 60 });
    clamp.extend();

    // get corner 
    arm->set_state(ArmState::LADDER_TOUCH);
    intake->set_state(IntakeState::REVERSE);
    drive_controller->boomerang(pose(144-24, 24, -145_deg), { lead: 0.4, threshold: 1.5, min_speed: 25 } );
    drive_controller->drive_time(-100, 100);
    intake->set_state(IntakeState::HOOD);
    drive_controller->face_angle(132_deg, { threshold: 2_deg });
    drive_controller->drive_time(100, 700);
    pros::delay(250);
    drive_controller->drive(-5, {min_speed: 40, chained: true });

    // touch ladder
    drive_controller->boomerang(pos(144-30, 72-36), { backwards: true, min_speed: 40 } );
    drive_controller->face_angle(-60_deg);
    drive_controller->drive_time(80, 650);
    drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);

}