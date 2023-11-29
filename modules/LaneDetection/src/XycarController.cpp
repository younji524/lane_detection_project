/**
 * @file XycarController.cpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the XycarController class for Xycar control in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
// User defined header
#include "XycarController.hpp"

namespace XyCar
{
XycarController::XycarController(const YAML::Node &config)
{
  set_configuration(config);
}

void XycarController::set_configuration(const YAML::Node &config)
{
  k_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<PREC>();
  k_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<PREC>();
  k_up_step_speed_ = config["XYCAR"]["UP_STEP_SPEED"].as<PREC>();
  k_down_step_speed_ = config["XYCAR"]["DOWN_STEP_SPEED"].as<PREC>();
  speed_ = config["XYCAR"]["START_SPEED"].as<PREC>();
}

xycar_msgs::xycar_motor XycarController::control(XyCar::PREC angle)
{
  // publisher_.publish(make_motor_message(angle, decide_speed(angle)));
  return make_motor_message(angle, decide_speed(angle));
}

PREC XycarController::decide_speed(PREC angle)
{
  // when xycar turn
  if (std::abs(angle) > 13)
  {
    speed_ -= k_down_step_speed_;
    speed_ = std::max(speed_, k_min_speed_);
  }
  // when xycar go straight
  else
  {
    speed_ += k_up_step_speed_;
    speed_ = std::min(speed_, k_max_speed_);
  }

  return speed_;
}

xycar_msgs::xycar_motor XycarController::make_motor_message(PREC angle, PREC speed)
{
  xycar_msgs::xycar_motor motor_message;
  motor_message.header.stamp = ros::Time::now();

  if (angle > 25.0)
    angle = 50;

  else if (angle < -25.0)
    angle = -50;

  // else if (std::abs(angle) > 13) angle *= 1.1;
  motor_message.angle = std::round(angle);
  motor_message.speed = std::round(speed);

  return motor_message;
}
} // namespace XyCar
