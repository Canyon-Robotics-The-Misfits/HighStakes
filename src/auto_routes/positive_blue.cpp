#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::positive_blue)
{
    odometry->setRotation(180_deg);
    odometry->setPosition(lib15442c::Vec(35, 15.5)); // TODO: update start position

    // std::cout << "POSITIVE BLUE" << std::endl;

    // Rush neutral mobile goal
    clamp.retract();
    oinker.retract();
    intake->set_redirect_mode(mechanism::IntakeRedirectMode::ALL);
    arm->move(80);

    drive_controller->boomerang(pose(48, 48, 210_deg), {backwards : true, lead : 0.4, threshold : 7, max_speed : 100, weird_offset: 270_deg_raw });
    arm->move(-80);
    drive_controller->drive_time(-25, 70);
    clamp.extend();
    pros::delay(250);
    drive_controller->drive_time(100, 150);
    arm->set_target(mechanism::ArmTarget::ALLIANCE_STAKE);
    drive_controller->facePoint(lib15442c::Vec(64, 16), 0_deg_raw, {threshold : 5_deg_raw});
    pros::delay(300);
    pros::delay(2000); // TODO: remove

    // Score on alliance stake
    drive_controller->boomerang(pos(68, 16), { max_speed : 100 });
    drive_controller->facePoint(lib15442c::Vec(72, 0), 15_deg_raw);
    drivetrain->move(-50, 0);
    pros::delay(50);
    drivetrain->move(0, 0);
    arm->move(-127);
    pros::delay(250);
    arm->move(-127);
    pros::delay(100);
    arm->move(127);
    pros::delay(200);
    arm->move(0);

    // Intake ring
    arm->set_target(mechanism::ArmTarget::NEUTRAL_STAKE);
    intake->set_redirect_mode(mechanism::IntakeRedirectMode::RED);
    drive_controller->drive_time(-127, 400);
    pros::delay(100);
    intake->move(127);
    drive_controller->facePoint(lib15442c::Vec(24, 54), 0_deg_raw, {threshold : 3_deg_raw});
    // drive_controller->boomerang(pose(24, 40, 290_deg), {threshold : 5, min_speed : 60});
    drive_controller->drive(odometry->getPose().vec().distance_to(lib15442c::Vec(24, 48)) - 13);
    drive_controller->drive_time(50, 75);
    drivetrain->move(0, 0);
    pros::delay(200);

    // get another ring
    // drive_controller->drive_time(-100, 200);
    // drive_controller->facePoint(lib15442c::Vec(72, 24), 0_deg_raw, {threshold: 5_deg_raw});
    // drive_controller->boomerang(pos(72, 24), { min_speed: 60 });
    // drive_controller->drive_time(50, 400);

    // Touch ladder
    // arm->set_target(mechanism::ArmTarget::LADDER_TOUCH);
    // // drive_controller->boomerang(pos(144 - (72 + 36 + 12), 36 - 12), {backwards : true, threshold : 5});
    // drive_controller->facePoint(lib15442c::Vec(144 - (72 + 36 - 12), 36 + 12));
    // drive_controller->drive(odometry->getPose().vec().distance_to(lib15442c::Vec(144 - (72 + 36 + 12), 36 - 12)));
    // drive_controller->faceAngle(35_deg, {threshold : 7_deg_raw});
    // intake->move(0);
    // drive_controller->drive_time(127, 300);
    // clamp.retract();
    // drivetrain->set_brake_mode(lib15442c::MotorBrakeMode::COAST);
    // arm->move(0);
}