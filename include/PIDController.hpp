#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd);
    double compute(double target, double actual);
    void reset();

private:
    double Kp, Ki, Kd;
    double error, integral, derivative;
    double prev_error;
};

#endif
