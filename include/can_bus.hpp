#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace eae {

constexpr std::size_t kCanMaxDlc = 8;

struct CanFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, kCanMaxDlc> data{};
};

// In-process CAN bus simulator. There is no real transceiver here - this
// models a single shared bus as a FIFO: send() queues a frame, receive()
// dequeues the next one, exactly like reading the next frame off a CAN
// controller's receive mailbox.
class CanBus {
public:
    void send(const CanFrame& frame);
    std::optional<CanFrame> receive();
    bool empty() const;
    std::size_t size() const;

private:
    std::deque<CanFrame> queue_;
};

} // namespace eae
