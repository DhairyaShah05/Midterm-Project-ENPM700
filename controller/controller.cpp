#include "controller.hpp"

Controller::Controller(Vehicle vehicle)
    : vehicle_(vehicle), targetHeading_(0.0) {}

void Controller::setTargetHeading(double targetHeading) {
  targetHeading_ = targetHeading;
  vehicle_.update(targetHeading_);
}
