#include "pid_controller.hpp"

namespace eae {

PidController::PidController(double kp, double ki, double kd, double output_min, double output_max,
                              Action action)
    : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max), action_(action) {}

double PidController::update(double setpoint, double measurement, double dt_seconds) {
    double error = (action_ == Action::Direct) ? (measurement - setpoint) : (setpoint - measurement);

    double proportional = kp_ * error;

    double derivative = 0.0;
    if (has_previous_error_ && dt_seconds > 0.0) {
        derivative = kd_ * (error - previous_error_) / dt_seconds;
    }

    // Compute the integral term against a tentative accumulator so we can
    // decide below whether to actually commit it (anti-windup).
    double tentative_integral = integral_ + error * dt_seconds;
    double output = proportional + ki_ * tentative_integral + derivative;

    if (output > output_max_) {
        output = output_max_;
    } else if (output < output_min_) {
        output = output_min_;
    } else {
        // Only accumulate the integral while the output is not saturated,
        // otherwise a long-standing error would "wind up" the integral term
        // far past what's needed and cause overshoot once it recovers.
        integral_ = tentative_integral;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    return output;
}

void PidController::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_error_ = false;
}

} // namespace eae
