#ifndef LANE_DETECTION__COMMON_HPP
#define LANE_DETECTION__COMMON_HPP

// System header
#include <cstdint>
// Third party header
#include <yaml-cpp/yaml.h>

namespace XyCar
{
using PREC = double; ///< PRECISION Data type for maintenance.

/**
 * @details  A structure that represents the state of the lanes.
 */
struct State {
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
