#pragma once

#include <array>
#include <cstdlib>
#include <iostream>
#include <string>

inline bool lzh_ros_environment_is_valid()
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  if (domain == nullptr || std::string(domain) != "42") {
    std::cerr << "ROS_DOMAIN_ID must be 42; got "
              << (domain == nullptr ? "unset" : domain) << std::endl;
    return false;
  }

  const std::array<const char *, 5> path_variables = {
    "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
    "PYTHONPATH", "LD_LIBRARY_PATH"
  };
  const std::array<std::string, 3> forbidden = {
    "g1d_ros2_ws", "g1d_ros2_yqqh", "unitree_ros_official"
  };
  for (const auto * variable : path_variables) {
    const char * raw = std::getenv(variable);
    if (raw == nullptr) {
      continue;
    }
    const std::string value(raw);
    for (const auto & marker : forbidden) {
      if (value.find(marker) != std::string::npos) {
        std::cerr << variable << " contains forbidden overlay path marker: "
                  << marker << std::endl;
        return false;
      }
    }
  }
  return true;
}
