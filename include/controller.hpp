/**
 * @file controller.hpp
 * @brief Provides an interface for controlling vehicle behavior.
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "Vehicle.hpp"

/**
 * @class Controller
 * @brief Manages the vehicle and its navigation.
 */
class Controller {
 public:
  /**
   * @brief Constructor to initialize the Controller with a Vehicle.
   * @param vehicle The Vehicle instance to control.
   */
  Controller(Vehicle vehicle);

  /**
   * @brief Sets a new target heading for the vehicle.
   * @param targetHeading The desired heading for the vehicle.
   */
  void setTargetHeading(double targetHeading);

 private:
  Vehicle vehicle_;  ///< Vehicle instance to control
  double targetHeading_;  ///< Desired heading for the vehicle
};

#endif  // CONTROLLER_HPP_
