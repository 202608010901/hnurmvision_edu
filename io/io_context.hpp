#pragma once
#include "config.hpp"
#include "types.hpp"
#include <optional>

namespace training::io {
struct IoContext {
  Config config;
  void open();
  std::optional<SensorData> read();
  void write(const Command& command);
  void close();
};
}  // namespace training::io
