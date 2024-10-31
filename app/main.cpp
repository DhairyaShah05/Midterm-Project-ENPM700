#include "AckermannController.hpp"
#include "controller.hpp"
#include "PIDController.hpp"
#include "Vehicle.hpp"
#include <iostream>
#include <cstdlib>  // For random numbers
#include <ctime>    // For seeding random number generator

int main() {
    // Initialize random seed
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Define PID controller gains and max steering angle
    double Kp = 1.0;
    double Ki = 0.1;
    double Kd = 0.05;
    double maxSteeringAngle = 30.0;  // Maximum allowable steering angle in degrees

    // Instantiate an AckermannController with PID parameters and steering limit
    AckermannController ackermannController(Kp, Ki, Kd, maxSteeringAngle);

    // Create a Vehicle controlled by the AckermannController
    Vehicle vehicle(ackermannController);

    // Initialize the main Controller with the vehicle
    Controller controller(vehicle);

    // Loop to test 5 random target headings
    for (int i = 0; i < 5; ++i) {
        // Generate a random target heading within the range -45 to 45 degrees
        double targetHeading = std::rand() % 90 - 45;
        controller.setTargetHeading(targetHeading);
        std::cout << "Random target heading set to: " << targetHeading << " degrees" << std::endl;

        // Simulate updating the vehicle’s heading
        vehicle.update(targetHeading);

        // Print out the current heading of the vehicle
        std::cout << "Vehicle heading after update: " << targetHeading << " degrees" << std::endl;
    }

    return 0;
}
