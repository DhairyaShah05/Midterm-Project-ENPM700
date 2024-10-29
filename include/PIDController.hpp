/**
 * @file PIDController.hpp
 * @brief Defines the PIDController class for computing control outputs.
 */

#ifndef PIDCONTROLLER_HPP_
#define PIDCONTROLLER_HPP_

/**
 * @class PIDController
 * @brief A Proportional-Integral-Derivative controller to compute control values.
 */
class PIDController {
 public:
  /**
   * @brief Constructor to initialize the PID controller with gains.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  PIDController(double Kp, double Ki, double Kd);

  /**
   * @brief Computes the control output based on target and actual values.
   * @param target The desired target value.
   * @param actual The actual current value.
   * @return The computed control output.
   */
  double compute(double target, double actual);

  /**
   * @brief Resets the integral and derivative terms.
   */
  void reset();

 private:
  double Kp_;  ///< Proportional gain
  double Ki_;  ///< Integral gain
  double Kd_;  ///< Derivative gain
  double error_;  ///< Error between target and actual
  double integral_;  ///< Accumulated integral
  double derivative_;  ///< Derivative of error
  double prev_error_;  ///< Previous error for derivative calculation
};

#endif  // PIDCONTROLLER_HPP_
