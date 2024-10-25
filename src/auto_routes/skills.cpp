#include "main.h"
#include "autonomous.h"


AUTO_ROUTE(auto_routes::skills)
{
    odometry->setRotation(-30_deg + 180_deg);
    odometry->setPosition(lib15442c::Vec(53.5, 13));
    
    // pickup mogo
    drive_controller->drive_time(-60, 50);
    clamp.extend();
    drive_controller->drive_time(-80, 100);
    pros::delay(100);

    // intake rings
    ring_mech->set_state(mechanism::INTAKE_HOOD);
    drive_controller->boomerang(pos(48, 48));
    drive_controller->boomerang(pose(24, 48, -135_deg), { lead: 0.4 });
    drive_controller->facePoint(pos(27, 24).vec(), 0_deg, { threshold: 5_deg });
    drive_controller->boomerang(pose(27, 24, 180_deg), { lead: 0.8 });
    drive_controller->drive_time(80, 300);
    drive_controller->drive_time(-100, 100);
    drive_controller->facePoint(pos(12, 24).vec(), 0_deg, { threshold: 5_deg });
    drive_controller->drive_time(100, 300);
    
    // dropoff mogo
    drive_controller->facePoint(pos(0, 0).vec(), 180_deg, { threshold: 10_deg });
    clamp.retract();
    drive_controller->drive_time(-127, 400);
    pros::delay(200);
    drive_controller->drive_time(127, 300);

    // pickup next mogo
    drive_controller->drive_to(pose(144-48, 24, 90_deg), { backwards: true, threshold: 6 });
    clamp.extend();
    drive_controller->drive_time(-100, 100);
    pros::delay(150);

    // intake rings
    drive_controller->boomerang(pos(144-48, 48));
    drive_controller->boomerang(pose(144-24, 48, 20_deg), { lead: 0.4 });
    drive_controller->boomerang(pos(144-12, 72), { threshold: 2 });
    drive_controller->drive_time(-100, 250);
    drive_controller->facePoint(pos(144-27, 24).vec(), 0_deg, { threshold: 5_deg });
    drive_controller->boomerang(pose(144-27, 24, 180_deg), { lead: 0.8 });
    drive_controller->drive_time(80, 300);
    drive_controller->drive_time(-100, 100);
    drive_controller->facePoint(pos(144-12, 24).vec(), 0_deg, { threshold: 5_deg });
    drive_controller->drive_time(100, 300);
    
    // dropoff mogo
    drive_controller->facePoint(pos(144, 0).vec(), 180_deg, { threshold: 10_deg });
    clamp.retract();
    drive_controller->drive_time(-127, 400);
    pros::delay(200);
    drive_controller->drive_time(127, 300);

}