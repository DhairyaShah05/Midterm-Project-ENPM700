/**
 * @file Vehicle.hpp
 * @brief Defines the Vehicle class, representing a vehicle controlled by AckermannController.
 */

#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "AckermannController.hpp"

/**
 * @class Vehicle
 * @brief Represents a vehicle with a heading controlled by an AckermannController.
 */
class Vehicle {
 public:
  /**
   * @brief Constructor to initialize the Vehicle with a given controller.
   * @param controller AckermannController instance for steering control.
   */
  Vehicle(AckermannController controller);

  /**
   * @brief Updates the vehicle's heading based on the target heading.
   * @param targetHeading The desired heading for the vehicle.
   */
  void update(double targetHeading);

 private:
  AckermannController controller_;  ///< Ackermann controller for steering
  double heading_;  ///< Current heading of the vehicle
};

#endif  // VEHICLE_HPP_
