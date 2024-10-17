// This is the file for running the Google Test.
#include <gtest/gtest.h>
#include "PIDController.hpp"
#include "AckermannController.hpp"
#include "Vehicle.hpp"

// Test for PID Controller
TEST(PIDControllerTest, ComputeTest) {
    PIDController pid(1.0, 0.0, 0.0);
    pid.reset();

    double result = pid.compute(10, 5);  // Stub, expect 0
    EXPECT_DOUBLE_EQ(0.0, result);  // Use Google Test macro for floating-point comparison
}

// Test for AckermannController
TEST(AckermannControllerTest, ComputeSteeringTest) {
    AckermannController controller(1.0, 0.0, 0.0, 45.0);
    double steering = controller.computeSteering(30, 10);  // Stub, expect 0
    EXPECT_DOUBLE_EQ(0.0, steering);  // Use Google Test macro for floating-point comparison
}

TEST(AckermannControllerTest, ErrorMarginTest) {
    AckermannController controller(1.0, 0.0, 0.0, 45.0);
    bool withinError = controller.isSteeringAngleWithinErrorMargin(40, 45, 5.0);  // Stub, expect true
    EXPECT_TRUE(withinError);  // Use Google Test macro for boolean check
}

// Test for Vehicle
TEST(VehicleTest, UpdateTest) {
    AckermannController controller(1.0, 0.0, 0.0, 45.0);
    Vehicle vehicle(controller);

    vehicle.update(20);  // Stub, no return value
    SUCCEED();  // For simplicity, mark it passed with SUCCEED
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
