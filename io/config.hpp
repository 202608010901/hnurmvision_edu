#pragma once
#include "types.hpp"
#include <array>
#include <string>
namespace training {
struct Config {
  bool display = true;
  bool allow_fire = false;
  std::array<double, 9> camera_matrix{
      1303.6753236814714, 0, 720, 0, 1303.6753236814714, 540, 0, 0, 1};
  std::array<double, 5> distort_coeffs{0, 0, 0, 0, 0};
  std::array<double, 9> camera_to_gimbal{
      0, 0.4226182617, 0.9063077870, -1, 0, 0,
      0, -0.9063077870, 0.4226182617};
  std::array<double, 3> camera_to_gimbal_translation{0, 0, 0};
};
Config load_config(const std::string& path);
CameraCalibration load_calibration(const Config& config);
}
