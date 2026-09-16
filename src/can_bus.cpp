#include "can_bus.hpp"

namespace eae {

void CanBus::send(const CanFrame& frame) {
    queue_.push_back(frame);
}

std::optional<CanFrame> CanBus::receive() {
    if (queue_.empty()) {
        return std::nullopt;
    }

    CanFrame frame = queue_.front();
    queue_.pop_front();
    return frame;
}

bool CanBus::empty() const {
    return queue_.empty();
}

std::size_t CanBus::size() const {
    return queue_.size();
}

} // namespace eae
