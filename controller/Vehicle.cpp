#include "Vehicle.hpp"

Vehicle::Vehicle(AckermannController controller)
    : controller(controller), heading(0) {}

void Vehicle::update(double targetHeading) {
    // Logic here
}

