#ifndef LANE_DETECTION__XYCARCONTROLLER_HPP
#define LANE_DETECTION__XYCARCONTROLLER_HPP

// system header
#include <algorithm>
#include <cmath>

// third party header
#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

// user defined header
#include "Common.hpp"

namespace XyCar {
class XycarController {
public:
  using Ptr = XycarController *;
  XycarController(const YAML::Node &config) { set_configuration(config); }

  /**
   * @details Transmit the speed & angle to xycar motor driver.
   * @param[in] angle Steering angle of xycar.\n (-20~20[deg] are mapped to
   * -50~50 values)
   * @return void
   */
  xycar_msgs::xycar_motor control(PREC angle);

private:
  // ros::NodeHandle node_handler_;
  // ros::Publisher publisher_;
  PREC k_max_speed_;
  PREC k_min_speed_;
  PREC k_up_step_speed_;
  PREC k_down_step_speed_;
  PREC speed_;

  /**
   * @details set values from config
   * @param[in] config config.yaml file
   * @return void
   */
  void set_configuration(const YAML::Node &config);

  /**
   * @details Decide the speed of xycar based on the 'angle' value.
   * @param[in] angle Steering angle of xycar.\n (-20~20[deg] are mapped to
   * -50~50 values)
   * @return XyCar::PREC
   */
  PREC decide_speed(PREC angle);

  /**
   * @details Create a xycar_motor message used to drive xycar.
   * @param angle Steering angle of xycar.\n (-20~20[deg] are mapped to -50~50
   * values)
   * @param speed Speed of xycar.
   * @return xycar_msgs::xycar_motor
   */
  xycar_msgs::xycar_motor make_motor_message(PREC angle, PREC speed);
};
} // namespace XyCar

#endif // LANE_DETECTION__XYCARCONTROLLER_HPP
