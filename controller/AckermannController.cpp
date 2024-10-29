#include "AckermannController.hpp"
#include <cmath>  // Include cmath for std::abs

AckermannController::AckermannController(double Kp, double Ki, double Kd, double maxSteeringAngle)
    : headingController_(Kp, Ki, Kd), maxSteeringAngle_(maxSteeringAngle) {}

double AckermannController::computeSteering(double targetHeading, double currentHeading) {
  double steeringAngle = headingController_.compute(targetHeading, currentHeading);
  if (steeringAngle > maxSteeringAngle_) {
    return maxSteeringAngle_;
  } else if (steeringAngle < -maxSteeringAngle_) {
    return -maxSteeringAngle_;
  }
  return steeringAngle;
}

bool AckermannController::isSteeringAngleWithinErrorMargin(double actualAngle, double targetAngle, double margin) {
  return std::abs(actualAngle - targetAngle) <= margin;
}
