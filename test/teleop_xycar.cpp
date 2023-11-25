// system header
#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>
// third party header
#include "ros/ros.h"
// user defined header
#include "xycar_msgs/xycar_motor.h"

/**
 * @brief Namespace for global variables
 */
namespace
{
  int32_t speed = 0;
  int32_t angle = 0;
  std::atomic<bool> is_running{true};
}

/**
 * @details Control the xycar according to key input.
 * Press 'i' to increase the forward speed.
 * Press 'k' to decrease the forward speed.
 * Press 'j' to turn the steering to the left.
 * Press 'l' to turn the steering to the right.
 * @return void
 */
void inputThread();

int32_t main(int32_t argc, char **argv)
{
  ros::init(argc, argv, "teleop_xycar", ros::init_options::AnonymousName);
  ros::NodeHandle node_handler;
  ros::Publisher publisher = node_handler.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1);

  std::thread input_thread(inputThread);

  while (ros::ok() && is_running)
  {
    xycar_msgs::xycar_motor motor_message;
    motor_message.speed = speed;
    motor_message.angle = angle;
    publisher.publish(motor_message);

    ros::spinOnce();
  }

  input_thread.join();

  return 0;
}

void inputThread()
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
