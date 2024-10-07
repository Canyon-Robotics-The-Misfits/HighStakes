#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::positive_red)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(144-35, 15.5)); // TODO: update start position

    // Rush neutral mobile goal
    clamp.retract();
    oinker.retract();
    intake->set_redirect_mode(mechanism::IntakeRedirectMode::ALL);
    arm->move(80);

    drive_controller->boomerang(pose(144 - 48, 48, 150_deg), { backwards: true, lead: 0.4, threshold: 7, max_speed: 100 });
    arm->move(-80);
    drive_controller->drive_time(-25, 70);
    clamp.extend();
    pros::delay(250);
    drive_controller->drive_time(100, 150);
    arm->set_target(mechanism::ArmTarget::ALLIANCE_STAKE);
    drive_controller->facePoint(lib15442c::Vec(80, 26), 0_deg, { threshold: 5_deg });
    pros::delay(300);

    // Score on alliance stake
    drive_controller->boomerang(pos(80.5, 16), { max_speed: 100 });
    drive_controller->faceAngle(195_deg, { threshold: 2_deg, });
    drivetrain->move(30, 0);
    pros::delay(50);
    drivetrain->move(0, 0);
    arm->move(-127);
    pros::delay(250);
    arm->move(-127);
    pros::delay(100);
    arm->move(127);
    pros::delay(100);
    arm->move(0);

    // Intake ring
    intake->set_redirect_mode(mechanism::IntakeRedirectMode::NONE);
    drive_controller->drive_time(-127, 400);
    pros::delay(100);
    intake->move(127);
    drive_controller->facePoint(lib15442c::Vec(120, 48), 0_deg, { threshold: 10_deg });
    drive_controller->boomerang(pos(120, 40), { threshold: 3, min_speed: 60 });
    drive_controller->drive_time(50, 100);
    drivetrain->move(0, 0);
    pros::delay(200);

    // get to corner
    arm->set_target(mechanism::ArmTarget::NEUTRAL_STAKE);
    drivetrain->move(-127, 10);
    pros::delay(400);
    drivetrain->move(0, 0);
    pros::delay(150);
    drive_controller->faceAngle(170_deg, { threshold: 5_deg });
    drive_controller->boomerang(pose(120, 16, 120_deg), { threshold: 3 });

    // Clear corner
    intake->move(0);
    drive_controller->faceAngle(100_deg, { threshold: 5_deg });
    oinker.extend();
    pros::delay(200);
    drivetrain->move(80, 0);
    pros::delay(350);
    drivetrain->move(0, 0);
    pros::delay(200);
    drivetrain->move(0, -127);
    pros::delay(400);
    drivetrain->move(0, 0);
    oinker.retract();

    intake->move(127);
    intake->set_redirect_mode(mechanism::IntakeRedirectMode::NONE);
    drive_controller->faceAngle(120_deg, { threshold: 5_deg });
    drivetrain->move(100, 0);
    pros::delay(350);
    drivetrain->move(-127, 0);
    pros::delay(200);
    drivetrain->move(0, 0);
    drive_controller->faceAngle(170_deg, { threshold: 5_deg });

    // Touch ladder
    // arm->set_target(mechanism::ArmTarget::LADDER_TOUCH);
    // drive_controller->boomerang(pos(72 + 36 + 12, 36 - 12), { backwards: true, threshold: 5 });
    // drive_controller->faceAngle(325_deg, { threshold: 7_deg });
    // intake->move(0);
    // drive_controller->drive_time(127, 450);
    // clamp.retract();
    // drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
    // arm->move(0);
}