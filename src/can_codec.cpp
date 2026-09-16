#include "can_codec.hpp"

namespace eae {

CanFrame encode_sensor_frame(const SensorFrame& frame) {
    CanFrame can{};
    can.id = can_id::kSensorFrame;
    can.dlc = 3;

    auto temp_fixed = static_cast<int16_t>(frame.coolant_temp_c * 10.0f);
    can.data[0] = static_cast<uint8_t>(temp_fixed & 0xFF);
    can.data[1] = static_cast<uint8_t>((temp_fixed >> 8) & 0xFF);

    uint8_t flags = 0;
    if (frame.ignition_on) {
        flags |= 0x01;
    }
    if (frame.level_ok) {
        flags |= 0x02;
    }
    can.data[2] = flags;

    return can;
}

SensorFrame decode_sensor_frame(const CanFrame& can) {
    SensorFrame frame{};

    auto temp_fixed = static_cast<int16_t>(static_cast<uint16_t>(can.data[0]) |
                                            (static_cast<uint16_t>(can.data[1]) << 8));
    frame.coolant_temp_c = static_cast<float>(temp_fixed) / 10.0f;
    frame.ignition_on = (can.data[2] & 0x01) != 0;
    frame.level_ok = (can.data[2] & 0x02) != 0;

    return frame;
}

CanFrame encode_actuator_frame(const ActuatorFrame& frame) {
    CanFrame can{};
    can.id = can_id::kActuatorFrame;
    can.dlc = 2;
    can.data[0] = frame.pump_run ? 1 : 0;
    can.data[1] = frame.fan_percent;
    return can;
}

ActuatorFrame decode_actuator_frame(const CanFrame& can) {
    ActuatorFrame frame{};
    frame.pump_run = can.data[0] != 0;
    frame.fan_percent = can.data[1];
    return frame;
}

} // namespace eae
