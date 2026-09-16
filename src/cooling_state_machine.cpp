#include "cooling_state_machine.hpp"

namespace eae {

const char* to_string(CoolingState state) {
    switch (state) {
        case CoolingState::Init:
            return "INIT";
        case CoolingState::Idle:
            return "IDLE";
        case CoolingState::Running:
            return "RUNNING";
        case CoolingState::Fault:
            return "FAULT";
    }
    return "UNKNOWN";
}

CoolingStateMachine::CoolingStateMachine(double setpoint_c, double kp, double ki, double kd)
    : setpoint_c_(setpoint_c),
      pid_(kp, ki, kd, /*output_min=*/0.0, /*output_max=*/100.0, PidController::Action::Direct) {}

bool CoolingStateMachine::is_sensor_valid(float temp_c) const {
    return temp_c >= kSensorMinValidC && temp_c <= kSensorMaxValidC;
}

CoolingOutput CoolingStateMachine::update(const SensorFrame& sensors, double dt_seconds) {
    CoolingOutput out;

    // Ignition off always returns the system to Idle and clears latched
    // faults - a manual fault-reset-on-power-cycle safety behavior that
    // forces an operator to notice and acknowledge a fault rather than
    // have it silently clear on its own.
    if (!sensors.ignition_on) {
        state_ = CoolingState::Idle;
        fault_latched_ = false;
        pid_.reset();

        out.state = state_;
        out.actuator = ActuatorFrame{false, 0};
        out.alarm = false;
        return out;
    }

    bool sensor_fault = !is_sensor_valid(sensors.coolant_temp_c);
    bool over_temp = !sensor_fault && sensors.coolant_temp_c >= kOverTempTripC;

    if (sensor_fault || over_temp) {
        fault_latched_ = true;
    }

    if (fault_latched_) {
        state_ = CoolingState::Fault;
    } else if (state_ == CoolingState::Init || state_ == CoolingState::Idle) {
        state_ = CoolingState::Running;
    }

    // Low coolant level inhibits the pump (protects against dry-run /
    // cavitation) but is not itself a latched fault - it clears as soon as
    // the level switch reports OK again, unlike over-temp/sensor faults.
    bool low_level_fault = !sensors.level_ok;
    bool pump_run = !low_level_fault;

    uint8_t fan_percent;
    if (state_ == CoolingState::Fault) {
        // Fail-active: force full cooling while a latched fault is present.
        fan_percent = 100;
    } else {
        double command = pid_.update(setpoint_c_, static_cast<double>(sensors.coolant_temp_c), dt_seconds);
        fan_percent = static_cast<uint8_t>(command + 0.5);
    }

    out.state = state_;
    out.actuator = ActuatorFrame{pump_run, fan_percent};
    out.alarm = (state_ == CoolingState::Fault) || low_level_fault;
    return out;
}

} // namespace eae
