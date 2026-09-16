#include <gtest/gtest.h>

#include "cooling_state_machine.hpp"

using namespace eae;

namespace {

CoolingStateMachine make_machine() {
    return CoolingStateMachine(/*setpoint_c=*/55.0, /*kp=*/4.0, /*ki=*/0.5, /*kd=*/0.1);
}

} // namespace

TEST(CoolingStateMachine, IgnitionOffStaysIdleWithOutputsOff) {
    CoolingStateMachine machine = make_machine();
    CoolingOutput out = machine.update({20.0f, false, true}, 1.0);

    EXPECT_EQ(out.state, CoolingState::Idle);
    EXPECT_FALSE(out.actuator.pump_run);
    EXPECT_EQ(out.actuator.fan_percent, 0);
    EXPECT_FALSE(out.alarm);
}

TEST(CoolingStateMachine, IgnitionOnTransitionsToRunningWithPumpOn) {
    CoolingStateMachine machine = make_machine();
    machine.update({20.0f, false, true}, 1.0);
    CoolingOutput out = machine.update({22.0f, true, true}, 1.0);

    EXPECT_EQ(out.state, CoolingState::Running);
    EXPECT_TRUE(out.actuator.pump_run);
    EXPECT_FALSE(out.alarm);
}

TEST(CoolingStateMachine, LowLevelInhibitsPumpButStaysRunning) {
    CoolingStateMachine machine = make_machine();
    machine.update({22.0f, true, true}, 1.0);
    CoolingOutput out = machine.update({30.0f, true, false}, 1.0);

    EXPECT_EQ(out.state, CoolingState::Running);
    EXPECT_FALSE(out.actuator.pump_run);
    EXPECT_TRUE(out.alarm);
}

TEST(CoolingStateMachine, OverTemperatureLatchesFaultAndForcesFanFull) {
    CoolingStateMachine machine = make_machine();
    machine.update({22.0f, true, true}, 1.0);
    CoolingOutput out = machine.update({85.0f, true, true}, 1.0);

    EXPECT_EQ(out.state, CoolingState::Fault);
    EXPECT_EQ(out.actuator.fan_percent, 100);
    EXPECT_TRUE(out.alarm);
}

TEST(CoolingStateMachine, FaultStaysLatchedUntilIgnitionCycled) {
    CoolingStateMachine machine = make_machine();
    machine.update({22.0f, true, true}, 1.0);
    machine.update({85.0f, true, true}, 1.0); // trips the fault

    CoolingOutput still_faulted = machine.update({50.0f, true, true}, 1.0); // temp back to normal
    EXPECT_EQ(still_faulted.state, CoolingState::Fault);

    CoolingOutput after_ignition_off = machine.update({30.0f, false, true}, 1.0);
    EXPECT_EQ(after_ignition_off.state, CoolingState::Idle);
    EXPECT_FALSE(after_ignition_off.alarm);

    CoolingOutput after_restart = machine.update({30.0f, true, true}, 1.0);
    EXPECT_EQ(after_restart.state, CoolingState::Running);
    EXPECT_FALSE(after_restart.alarm);
}

TEST(CoolingStateMachine, SensorOutOfRangeLatchesFault) {
    CoolingStateMachine machine = make_machine();
    machine.update({22.0f, true, true}, 1.0);
    CoolingOutput out = machine.update({200.0f, true, true}, 1.0);

    EXPECT_EQ(out.state, CoolingState::Fault);
    EXPECT_EQ(out.actuator.fan_percent, 100);
    EXPECT_TRUE(out.alarm);
}
