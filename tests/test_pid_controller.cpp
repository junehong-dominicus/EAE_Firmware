#include <gtest/gtest.h>

#include "pid_controller.hpp"

using namespace eae;

TEST(PidController, ProportionalRespondsToError) {
    PidController pid(2.0, 0.0, 0.0, 0.0, 100.0);
    double output = pid.update(/*setpoint=*/60.0, /*measurement=*/50.0, /*dt=*/1.0);
    EXPECT_NEAR(output, 20.0, 1e-9);
}

TEST(PidController, OutputClampsToUpperBound) {
    PidController pid(10.0, 0.0, 0.0, 0.0, 100.0);
    double output = pid.update(100.0, 0.0, 1.0);
    EXPECT_EQ(output, 100.0);
}

TEST(PidController, OutputClampsToLowerBound) {
    PidController pid(10.0, 0.0, 0.0, 0.0, 100.0);
    double output = pid.update(0.0, 100.0, 1.0);
    EXPECT_EQ(output, 0.0);
}

TEST(PidController, IntegralAccumulatesOverTime) {
    PidController pid(0.0, 1.0, 0.0, 0.0, 100.0);
    pid.update(10.0, 0.0, 1.0);               // error 10, integral -> 10
    double output = pid.update(10.0, 0.0, 1.0); // integral -> 20
    EXPECT_NEAR(output, 20.0, 1e-9);
}

TEST(PidController, ResetClearsIntegralAndDerivativeHistory) {
    PidController pid(0.0, 1.0, 0.0, 0.0, 100.0);
    pid.update(10.0, 0.0, 1.0);
    pid.reset();
    double output = pid.update(10.0, 0.0, 1.0);
    EXPECT_NEAR(output, 10.0, 1e-9); // integral restarted from zero
}

TEST(PidController, ConvergesTowardSetpointUnderConstantHeatLoad) {
    // Simple first-order plant: a constant heat load raises temperature,
    // and fan speed removes heat proportionally. The PID's integral term
    // should find the steady-state fan percentage that holds the
    // temperature at the setpoint despite the constant disturbance.
    const double setpoint = 55.0;
    const double heat_gain_c_per_s = 3.0;    // constant heat load (e.g. inverter dissipation)
    const double max_cooling_c_per_s = 10.0; // cooling rate at fan = 100%
    const double dt = 0.1;

    PidController pid(/*kp=*/5.0, /*ki=*/2.0, /*kd=*/0.5, /*output_min=*/0.0, /*output_max=*/100.0,
                       PidController::Action::Direct);

    double temp = 75.0;
    for (int i = 0; i < 600; ++i) {
        double fan_percent = pid.update(setpoint, temp, dt);
        temp += (heat_gain_c_per_s - max_cooling_c_per_s * (fan_percent / 100.0)) * dt;
    }

    EXPECT_NEAR(temp, setpoint, 1.0);
}
