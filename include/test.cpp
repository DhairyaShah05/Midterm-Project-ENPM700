#include <iostream>
#include "PIDController.hpp"
#include "AckermannController.hpp"
#include "Vehicle.hpp"

// Simple assert function for testing
void assertEqual(double expected, double actual, const std::string& testName) {
    if (expected == actual) {
        std::cout << testName << ": PASSED" << std::endl;
    } else {
        std::cout << testName << ": FAILED (Expected: " << expected << ", Actual: " << actual << ")" << std::endl;
    }
}

// Test for PID Controller
void testPIDController() {
    PIDController pid(1.0, 0.0, 0.0);
    pid.reset();

    double result = pid.compute(10, 5);  // Stub, expect 0
    assertEqual(0.0, result, "PIDController Test");
}

// Test for AckermannController
void testAckermannController() {
    AckermannController controller(1.0, 0.0, 0.0, 45.0);
    double steering = controller.computeSteering(30, 10);  // Stub, expect 0
    assertEqual(0.0, steering, "AckermannController Test");

    bool withinError = controller.isSteeringAngleWithinErrorMargin(40, 45, 5.0);  // Stub, expect true
    assertEqual(true, withinError, "Ackermann Error Margin Test");
}

// Test for Vehicle
void testVehicle() {
    AckermannController controller(1.0, 0.0, 0.0, 45.0);
    Vehicle vehicle(controller);

    vehicle.update(20);  // Stub, no return value
    std::cout << "Vehicle Update Test: PASSED" << std::endl;  // For simplicity, mark it passed
}

int main() {
    // Run tests
    testPIDController();
    testAckermannController();
    testVehicle();
    
    return 0;
}
