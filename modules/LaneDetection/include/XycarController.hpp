/**
 * @file XycarController.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the XycarController class for Xycar control in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__XYCARCONTROLLER_HPP
#define LANE_DETECTION__XYCARCONTROLLER_HPP

// System header
#include <algorithm>
#include <cmath>
// Third party header
#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"
// User defined header
#include "Common.hpp"

namespace XyCar
{
/**
 * @brief XycarController class for Xycar control.
 * @details This class is used to control a Xycar.
 */
class XycarController
{
public:
  using Ptr = XycarController *;  ///< Pointer type of class.
  /**
   * @brief Construct a new XycarController object.
   * @details This function constructs a new XycarController object.
   * @param[in] config The configuration of lane_detection project.
   */
  XycarController(const YAML::Node &config);

  /**
   * @brief Transmit the speed & angle to Xycar motor driver.
   * @details This function transmits the speed & angle to Xycar motor driver.
   * @param[in] angle The steering angle of Xycar. (-20~20[deg] are mapped to -50 ~ 50 values)
   * @return void
   */
  xycar_msgs::xycar_motor control(PREC angle);

private:
  // TODO: Node로 구현해보기
  // ros::NodeHandle node_handler_;
  // ros::Publisher publisher_;
  PREC k_max_speed_;  ///< The maximum speed of Xycar.
  PREC k_min_speed_;  ///< The minimum speed of Xycar.
  PREC k_up_step_speed_;  ///< The acceleration step of Xycar.
  PREC k_down_step_speed_;  ///< The deceleration step of Xycar.
  PREC speed_;  ///< The current speed of Xycar.

  /**
   * @brief Set values from config.
   * @details This function sets values from config.
   * @param[in] config The configuration of lane_detection project.
   * @return void
   */
  void set_configuration(const YAML::Node &config);

  /**
   * @brief Decide the speed of Xycar based on the 'angle' value.
   * @details This function decides the speed of Xycar based on the 'angle' value.
   * @param[in] angle The steering angle of Xycar. (-20~20[deg] are mapped to -50 ~ 50 values)
   * @return XyCar::PREC The speed of Xycar. (Here, PREC refers to double)
   */
  PREC decide_speed(PREC angle);

  /**
   * @brief Create a xycar_motor message used to drive Xycar.
   * @details This function creates a xycar_motor message used to drive Xycar.
   * @param angle The steering angle of Xycar. (-20 ~ 20[deg] are mapped to -50 ~ 50 values)
   * @param speed The speed of Xycar.
   * @return xycar_msgs::xycar_motor The topic message to Xycar.
   */
  xycar_msgs::xycar_motor make_motor_message(PREC angle, PREC speed);
};
} // namespace XyCar

#endif // LANE_DETECTION__XYCARCONTROLLER_HPP
