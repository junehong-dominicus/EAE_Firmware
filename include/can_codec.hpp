#pragma once

#include "can_bus.hpp"

namespace eae {

// CAN identifiers for the two frames on this bus - equivalent to entries in
// a DBC file for a real network.
namespace can_id {
constexpr uint32_t kSensorFrame = 0x100;   // ECU -> controller: temp, ignition, level
constexpr uint32_t kActuatorFrame = 0x200; // controller -> actuator drivers: pump, fan
} // namespace can_id

struct SensorFrame {
    float coolant_temp_c = 0.0f;
    bool ignition_on = false;
    bool level_ok = true;
};

struct ActuatorFrame {
    bool pump_run = false;
    uint8_t fan_percent = 0;
};

// Coolant temperature is packed as a signed 16-bit value in units of 0.1 C
// (a standard fixed-point CAN signal scaling), little-endian, in bytes 0-1.
// Byte 2 carries the ignition and level-switch digital inputs as bit flags.
CanFrame encode_sensor_frame(const SensorFrame& frame);
SensorFrame decode_sensor_frame(const CanFrame& frame);

// Byte 0 is the pump command, byte 1 is the fan command as a 0-100 percent.
CanFrame encode_actuator_frame(const ActuatorFrame& frame);
ActuatorFrame decode_actuator_frame(const CanFrame& frame);

} // namespace eae
