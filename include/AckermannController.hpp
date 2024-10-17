#ifndef ACKERMANNCONTROLLER_HPP
#define ACKERMANNCONTROLLER_HPP

#include "PIDController.hpp"

class AckermannController {
public:
    AckermannController(double Kp_h, double Ki_h, double Kd_h, double maxSteeringAngle);
    double computeSteering(double targetHeading, double currentHeading);
    bool isSteeringAngleWithinErrorMargin(double actualAngle, double targetAngle, double margin);

private:
    PIDController headingController;
    double maxSteeringAngle;
};

#endif

