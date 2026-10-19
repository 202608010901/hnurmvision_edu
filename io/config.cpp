#include "config.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace training {
namespace {
template <std::size_t N>
void read_array(const YAML::Node& root, const char* name,
                std::array<double, N>& output) {
  const auto node = root[name];
  if (!node || !node.IsSequence() || node.size() != N)
    throw std::runtime_error(std::string("配置项 ") + name + " 必须包含 " +
                             std::to_string(N) + " 个数");
  for (std::size_t i = 0; i < N; ++i) output[i] = node[i].as<double>();
  if (!std::all_of(output.begin(), output.end(),
                   [](double value) { return std::isfinite(value); }))
    throw std::runtime_error(std::string("配置项 ") + name + " 含非有限数值");
}
}

Config load_config(const std::string& path) {
  const auto y = YAML::LoadFile(path);
  Config c;
  if (y["display"]) c.display = y["display"].as<bool>();
  if (y["allow_fire"]) c.allow_fire = y["allow_fire"].as<bool>();
  read_array(y, "camera_matrix", c.camera_matrix);
  read_array(y, "distort_coeffs", c.distort_coeffs);
  read_array(y, "R_camera2gimbal", c.camera_to_gimbal);
  read_array(y, "t_camera2gimbal", c.camera_to_gimbal_translation);
  if (c.camera_matrix[0] <= 0 || c.camera_matrix[4] <= 0 ||
      std::abs(c.camera_matrix[8] - 1.0) > 1e-9)
    throw std::runtime_error("无效的仿真 IO 配置");
  return c;
}
}

namespace training {
CameraCalibration load_calibration(const Config& config) {
  CameraCalibration calibration;
  calibration.matrix = cv::Mat(config.camera_matrix).reshape(1, 3).clone();
  calibration.distortion = cv::Mat(config.distort_coeffs).reshape(1, 1).clone();
  calibration.camera_to_gimbal = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      config.camera_to_gimbal.data());
  calibration.camera_to_gimbal_translation = Eigen::Map<const Eigen::Vector3d>(
      config.camera_to_gimbal_translation.data());
  calibration.width = 1440;
  calibration.height = 1080;
  return calibration;
}
}
