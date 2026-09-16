#include <gtest/gtest.h>

#include "can_bus.hpp"

using namespace eae;

TEST(CanBus, ReceiveOnEmptyBusReturnsNullopt) {
    CanBus bus;
    EXPECT_TRUE(bus.empty());
    EXPECT_FALSE(bus.receive().has_value());
}

TEST(CanBus, SendThenReceiveReturnsSameFrame) {
    CanBus bus;
    CanFrame frame{};
    frame.id = 0x123;
    frame.dlc = 2;
    frame.data[0] = 0xAB;
    frame.data[1] = 0xCD;

    bus.send(frame);
    ASSERT_EQ(bus.size(), 1u);

    auto received = bus.receive();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->id, frame.id);
    EXPECT_EQ(received->dlc, frame.dlc);
    EXPECT_EQ(received->data[0], frame.data[0]);
    EXPECT_EQ(received->data[1], frame.data[1]);
    EXPECT_TRUE(bus.empty());
}

TEST(CanBus, FramesAreDeliveredInSendOrder) {
    CanBus bus;
    CanFrame first{};
    first.id = 0x100;
    CanFrame second{};
    second.id = 0x200;

    bus.send(first);
    bus.send(second);

    EXPECT_EQ(bus.receive()->id, 0x100u);
    EXPECT_EQ(bus.receive()->id, 0x200u);
}
