#include <cstdint>
#include <iostream>
#include <thread>
#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

int32_t speed = 0;
int32_t angle = 0;
bool isRunning = true;

void inputThread()
{
    char key;
    while (isRunning)
    {
        std::cin >> key;
        if (key == 'i') speed += 1;
        else if (key == 'k') speed -= 1;
        else if (key == 'j') angle -= 10;
        else if (key == 'l') angle += 10;
        else if (key == 27) isRunning = false; // ESC 키
    }
}

int32_t main(int32_t argc, char **argv)
{
    ros::init(argc, argv, "teleop_xycar", ros::init_options::AnonymousName);
    ros::NodeHandle node_handler;
    ros::Publisher publisher = node_handler.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1);

    std::thread input_thread(inputThread);

    while (ros::ok() && isRunning)
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
