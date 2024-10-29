#include "Vehicle.hpp"

Vehicle::Vehicle(AckermannController controller)
    : controller_(controller), heading_(0.0) {}

void Vehicle::update(double targetHeading) {
  double steeringAngle = controller_.computeSteering(targetHeading, heading_);
  if (!controller_.isSteeringAngleWithinErrorMargin(heading_, targetHeading, 0.5)) {
    heading_ += steeringAngle;
  }
}
