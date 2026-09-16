#pragma once

namespace eae {

// Standard PID controller with output clamping and conditional-integration
// anti-windup (the integral term only accumulates while the output is not
// saturated at a limit).
class PidController {
public:
    // Reverse: error = setpoint - measurement (output rises as PV falls
    //          below setpoint - e.g. a heater).
    // Direct:  error = measurement - setpoint (output rises as PV rises
    //          above setpoint - e.g. a cooling fan, which is what this
    //          project uses it for).
    enum class Action { Reverse, Direct };

    PidController(double kp, double ki, double kd, double output_min, double output_max,
                  Action action = Action::Reverse);

    // Advances the controller by one time step and returns the clamped
    // control output for the given setpoint/measurement pair.
    double update(double setpoint, double measurement, double dt_seconds);

    // Clears integral and derivative history, e.g. when the loop restarts.
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double output_min_;
    double output_max_;
    Action action_;

    double integral_ = 0.0;
    double previous_error_ = 0.0;
    bool has_previous_error_ = false;
};

} // namespace eae
