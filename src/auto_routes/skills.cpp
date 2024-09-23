#include "main.h"
#include "autonomous.h"

AUTO_ROUTE(auto_routes::skills)
{
    
    // drive_controller->faceAngle(90_deg, { timeout: INFINITY });
    // drive_controller->faceAngle(180_deg, { timeout: INFINITY });
    // drive_controller->faceAngle(0_deg, { timeout: INFINITY });
    // drive_controller->faceAngle(10_deg, { timeout: INFINITY });

    drive_controller->drive(24, { angle: 90_deg, timeout: INFINITY});

    
    // std::shared_ptr<lib15442c::DriveStraight> movement = std::make_shared<lib15442c::DriveStraight>(
    //     24,
    //     std::make_shared<lib15442c::PID>(lib15442c::PIDParameters { kP: 0, kI: 0, kD: 0 }),
    //     lib15442c::DriveParameters{},
    //     "AA"
    // );

    // std::shared_ptr<lib15442c::Face> movement = std::make_shared<lib15442c::Face>(
    //     lib15442c::FaceAngleTarget { angle: 0_deg },
    //     std::make_shared<lib15442c::PID>(lib15442c::PIDParameters { kP: 0, kI: 0, kD: 0 }),
    //     lib15442c::FaceParameters{},
    //     "AA"
    // );
}