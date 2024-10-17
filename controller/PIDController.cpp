#include "PIDController.hpp"

PIDController::PIDController(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), error(0), integral(0), derivative(0), prev_error(0) {}

double PIDController::compute(double target, double actual) {
    // Stub for error computation
    return 0.0;  // Stub: No actual implementation yet
}

void PIDController::reset() {
    // Stub for reset
}
