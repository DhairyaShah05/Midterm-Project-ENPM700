#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include "AckermannController.hpp"

class Vehicle {
public:
    Vehicle(AckermannController controller);
    void update(double targetHeading);

private:
    AckermannController controller;
    double heading;
};

#endif


