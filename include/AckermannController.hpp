/**
 * @file AckermannController.hpp
 * @brief Defines the AckermannController class for managing steering angle.
 */

#ifndef ACKERMANNCONTROLLER_HPP_
#define ACKERMANNCONTROLLER_HPP_

#include "PIDController.hpp"

/**
 * @class AckermannController
 * @brief Controls the steering angle for an Ackermann steering vehicle.
 */
class AckermannController {
 public:
  /**
   * @brief Constructor to initialize the AckermannController with PID gains and max steering angle.
   * @param Kp Proportional gain for heading controller.
   * @param Ki Integral gain for heading controller.
   * @param Kd Derivative gain for heading controller.
   * @param maxSteeringAngle Maximum allowed steering angle.
   */
  AckermannController(double Kp, double Ki, double Kd, double maxSteeringAngle);

  /**
   * @brief Computes the steering angle based on target and current headings.
   * @param targetHeading Desired heading.
   * @param currentHeading Current heading.
   * @return The computed steering angle.
   */
  double computeSteering(double targetHeading, double currentHeading);

  /**
   * @brief Checks if the steering angle is within the acceptable error margin.
   * @param actualAngle Current steering angle.
   * @param targetAngle Desired steering angle.
   * @param margin Acceptable margin of error.
   * @return True if within the margin, otherwise false.
   */
  bool isSteeringAngleWithinErrorMargin(double actualAngle, double targetAngle, double margin);

 private:
  PIDController headingController_;  ///< PID controller for heading adjustment
  double maxSteeringAngle_;  ///< Maximum steering angle limit
};

#endif  // ACKERMANNCONTROLLER_HPP_
