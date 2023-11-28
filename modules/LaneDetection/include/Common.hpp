/**
 * @file Common.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Provides the definition of the lane state structure and precision type alias in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-12
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__COMMON_HPP
#define LANE_DETECTION__COMMON_HPP

// System header
#include <cstdint>
// Third party header
#include "yaml-cpp/yaml.h"

namespace XyCar
{
using PREC = double; ///< PRECISION Data type for maintenance.

/**
 * @brief State struct that represents the state of the lanes.
 * @details This structure is used to store information about the state of the lanes.
 * State struct contains the slopes, intercept, positions of the left and right lanes
 * and a flag for stop line recognition.
 */
struct State
{
  PREC left_slope_;      ///< The slope of left lane.
  PREC left_intercept_;  ///< The intercept of left lane.
  PREC right_slope_;     ///< The slope of right lane.
  PREC right_intercept_; ///< The intercept of right lane.
  PREC left_pos_;        ///< The position of left lane.
  PREC right_pos_;       ///< The position of right lane.
  bool stop_flag_;       ///< The flag for stop line recognition.
};
} // namespace XyCar

#endif // LANE_DETECTION__COMMON_HPP
