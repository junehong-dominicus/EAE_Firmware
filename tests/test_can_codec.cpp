#include <gtest/gtest.h>

#include "can_codec.hpp"

using namespace eae;

TEST(CanCodec, SensorFrameRoundTrip) {
    SensorFrame original{56.7f, true, false};
    CanFrame can = encode_sensor_frame(original);

    EXPECT_EQ(can.id, can_id::kSensorFrame);
    EXPECT_EQ(can.dlc, 3);

    SensorFrame decoded = decode_sensor_frame(can);
    EXPECT_NEAR(decoded.coolant_temp_c, original.coolant_temp_c, 0.05f);
    EXPECT_EQ(decoded.ignition_on, original.ignition_on);
    EXPECT_EQ(decoded.level_ok, original.level_ok);
}

TEST(CanCodec, SensorFrameNegativeTemperature) {
    SensorFrame original{-12.3f, false, true};
    CanFrame can = encode_sensor_frame(original);
    SensorFrame decoded = decode_sensor_frame(can);

    EXPECT_NEAR(decoded.coolant_temp_c, original.coolant_temp_c, 0.05f);
}

TEST(CanCodec, ActuatorFrameRoundTrip) {
    ActuatorFrame original{true, 73};
    CanFrame can = encode_actuator_frame(original);

    EXPECT_EQ(can.id, can_id::kActuatorFrame);
    EXPECT_EQ(can.dlc, 2);

    ActuatorFrame decoded = decode_actuator_frame(can);
    EXPECT_EQ(decoded.pump_run, original.pump_run);
    EXPECT_EQ(decoded.fan_percent, original.fan_percent);
}
