/**
 * @file teleop_xycar.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the functions for controlling the XyCar based on key input.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
// System header
#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>
// Third party header
#include "ros/ros.h"
// User defined header
#include "xycar_msgs/xycar_motor.h"

namespace // Global variables
{
  int32_t speed = 0;
  int32_t angle = 0;
  std::atomic<bool> is_running{true};
}

/**
 * @brief Control the Xycar according to key input.
 * @details This function controls the Xycar according to key input.
 * If press 'i' to increase the forward speed.
 * If press 'k' to decrease the forward speed.
 * If press 'j' to turn the steering to the left.
 * If press 'l' to turn the steering to the right.
 * @return void
 */
void input_thread();

int32_t main(int32_t argc, char **argv)
{
  ros::init(argc, argv, "teleop_xycar", ros::init_options::AnonymousName);
  ros::NodeHandle node_handler;
  ros::Publisher publisher = node_handler.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1);

  std::thread control_thread(input_thread);

  while (ros::ok() && is_running)
  {
    xycar_msgs::xycar_motor motor_message;
    motor_message.speed = speed;
    motor_message.angle = angle;
    publisher.publish(motor_message);

    ros::spinOnce();
  }

  control_thread.join();

  return 0;
}

void input_thread()
{
  char key;
  while (is_running)
  {
    std::cin >> key;
    if (key == 'i')
      speed += 1;
    else if (key == 'k')
      speed -= 1;
    else if (key == 'j')
      angle -= 10;
    else if (key == 'l')
      angle += 10;
    else if (key == 27)  // ESC
      is_running.store(false);
  }
}
