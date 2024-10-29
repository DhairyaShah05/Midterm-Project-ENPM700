#include "PIDController.hpp"

PIDController::PIDController(double Kp, double Ki, double Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), integral_(0.0), prev_error_(0.0) {}

double PIDController::compute(double target, double actual) {
  error_ = target - actual;
  integral_ += error_;
  derivative_ = error_ - prev_error_;
  prev_error_ = error_;
  return Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative_;
}

void PIDController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
}
