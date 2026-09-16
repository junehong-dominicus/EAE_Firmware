#pragma once

#include "can_codec.hpp"
#include "pid_controller.hpp"

namespace eae {

enum class CoolingState { Init, Idle, Running, Fault };

const char* to_string(CoolingState state);

struct CoolingOutput {
    CoolingState state = CoolingState::Init;
    ActuatorFrame actuator{};
    bool alarm = false;
};

// State machine for the Section 5 cooling loop, restructured from the
// Section 7 on/off logic into explicit states and a PID-driven fan speed.
//
//   Init    -> Running   once ignition is on and no fault is present
//   Idle    <- any state whenever ignition is off (also clears faults)
//   Running -> Fault     on over-temperature trip or an implausible sensor
//                        reading (sensor fault); low coolant level inhibits
//                        the pump but does not itself force a Fault state
//   Fault   -> Idle      only when ignition is cycled off, matching a
//                        manual fault-acknowledge safety behavior
class CoolingStateMachine {
public:
    CoolingStateMachine(double setpoint_c, double kp, double ki, double kd);

    CoolingOutput update(const SensorFrame& sensors, double dt_seconds);

private:
    bool is_sensor_valid(float temp_c) const;

    double setpoint_c_;
    PidController pid_;
    CoolingState state_ = CoolingState::Init;
    bool fault_latched_ = false;

    static constexpr float kOverTempTripC = 80.0f;
    static constexpr float kSensorMinValidC = -40.0f;
    static constexpr float kSensorMaxValidC = 150.0f;
};

} // namespace eae
