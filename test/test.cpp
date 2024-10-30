/**
 * @file test_main.cpp
 * @brief Google Test cases for PIDController, AckermannController, Vehicle, and Controller classes.
 */

#include <gtest/gtest.h>
#include "PIDController.hpp"
#include "AckermannController.hpp"
#include "Vehicle.hpp"
#include "controller.hpp"  // Include the Controller class header

/**
 * @brief Tests the PIDController's compute function with a proportional gain.
 *
 * This test initializes a PID controller with Kp = 1.0, Ki = 0.0, and Kd = 0.0.
 * It computes the control output based on a target of 10 and actual value of 5,
 * expecting a result of 5.0 due to the proportional term.
 */
TEST(PIDControllerTest, ComputeTest) {
  PIDController pid(1.0, 0.0, 0.0);
  pid.reset();

  double result = pid.compute(10, 5);  // Expected result is 5.0
  EXPECT_DOUBLE_EQ(5.0, result);  // Assert the expected value
}

/**
 * @brief Tests the AckermannController's computeSteering function for within and out of bounds steering.
 *
 * This test initializes an Ackermann controller with Kp = 1.0, Ki = 0.0, Kd = 0.0,
 * and a max steering angle of 45.0 degrees. It checks various scenarios to ensure
 * the steering angle is capped correctly.
 */
TEST(AckermannControllerTest, ComputeSteeringBoundsTest) {
  AckermannController controller(1.0, 0.0, 0.0, 45.0);

  // Test within bounds
  double steering = controller.computeSteering(30, 10);  // Expected result is 20.0
  EXPECT_DOUBLE_EQ(20.0, steering);  // Assert the expected value
  
  // Test exceeding maxSteeringAngle
  steering = controller.computeSteering(100, 10);  // Computed steering angle will exceed 45.0
  EXPECT_DOUBLE_EQ(45.0, steering);  // Assert that it is capped at maxSteeringAngle (45.0)

  // Test exceeding -maxSteeringAngle
  steering = controller.computeSteering(-100, 10);  // Computed steering angle will be below -45.0
  EXPECT_DOUBLE_EQ(-45.0, steering);  // Assert that it is capped at -maxSteeringAngle (-45.0)
}

/**
 * @brief Tests if the AckermannController correctly identifies angles within the error margin.
 *
 * This test checks if an Ackermann controller correctly identifies that an
 * actual angle of 40 is within a margin of 5.0 degrees from the target angle of 45.
 */
TEST(AckermannControllerTest, ErrorMarginTest) {
  AckermannController controller(1.0, 0.0, 0.0, 45.0);
  bool within_error = controller.isSteeringAngleWithinErrorMargin(40, 45, 5.0);  // Expected result is true
  EXPECT_TRUE(within_error);  // Assert the expected boolean outcome
}

/**
 * @brief Tests the Vehicle's update function with a given target heading.
 *
 * This test initializes a Vehicle with an Ackermann controller and updates its
 * heading based on a target heading of 20. The test is marked as passed with
 * SUCCEED() to ensure it compiles and runs, but does not check specific outcomes.
 */
TEST(VehicleTest, UpdateTest) {
  AckermannController controller(1.0, 0.0, 0.0, 45.0);
  Vehicle vehicle(controller);

  vehicle.update(20);  // Perform update operation
  SUCCEED();  // Mark the test as successful
}

/**
 * @brief Tests the Controller's setTargetHeading function.
 *
 * This test initializes a Controller with a Vehicle and sets a target heading.
 * It checks if the function executes without errors.
 */
TEST(ControllerTest, SetTargetHeadingTest) {
  // Initialize the AckermannController with required PID gains and max steering angle
  AckermannController ackermann(1.0, 0.0, 0.0, 45.0);
  
  // Initialize Vehicle with AckermannController
  Vehicle vehicle(ackermann);

  // Initialize the Controller with the Vehicle
  Controller controller(vehicle);

  // Set a target heading and ensure it runs successfully
  double targetHeading = 30.0;
  controller.setTargetHeading(targetHeading);
  SUCCEED();  // Mark the test as successful
}

/**
 * @brief Main function for running all Google Test cases.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Result of running all tests.
 */
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
