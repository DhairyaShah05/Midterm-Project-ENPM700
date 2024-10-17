#include "AckermannController.hpp"

AckermannController::AckermannController(double Kp_h, double Ki_h, double Kd_h, double maxSteeringAngle)
    : headingController(Kp_h, Ki_h, Kd_h), maxSteeringAngle(maxSteeringAngle) {}

double AckermannController::computeSteering(double targetHeading, double currentHeading) {
    // Stub
    return 0.0;
}

bool AckermannController::isSteeringAngleWithinErrorMargin(double actualAngle, double targetAngle, double margin) {
    // Stub
    return true;
}

