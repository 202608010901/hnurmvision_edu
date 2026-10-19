#include "io_context.hpp"
#include <chrono>
#include <thread>

namespace training::io {
void IoContext::open() {
}

std::optional<SensorData> IoContext::read() {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  return std::nullopt;
}

void IoContext::write(const Command& command) {
  (void)command;
}

void IoContext::close() {
}
}  // namespace training::io
