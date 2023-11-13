#include <cstdint>
#include <iostream>
#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

enum KeyInput
{
    UP = 72,
    DOWN = 80,
    LEFT = 75,
    RIGHT = 77
};

int32_t main(int32_t argc, char **argv)
{
    ros::init(argc, argv, "teleop_xycar", ros::init_options::AnonymousName);
    ros::NodeHandle node_handler;
    ros::Publisher publisher = node_handler.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1);

    int32_t speed = 0;
    int32_t angle = 0;
    char key;

    constexpr int32_t k_max_speed = 20;
    constexpr int32_t k_min_speed = 0;
    constexpr int32_t k_max_angle = 50;
    constexpr int32_t k_min_angle = -50;

    std::cout << "Use 'i' to increase speed\n'k' to decrease speed\n'j' to turn left\n'l' to turn right\n";

    while(ros::ok())
    {
        std::cin >> key;

        // i, k : control speed
        if (key == 'i') // forward
        {
            speed += 5;
            speed = std::min(speed, k_max_speed);
        }
        else if (key == 'k') // backward
        {
            speed -= 5;
            speed = std::max(speed, k_min_speed);
        }

        // j, l : steering control
        if (key == 'j') // left
        {
            angle -= 10;
            angle = std::max(k_min_angle, std::min(angle, k_max_angle));
        }
        else if (key == 'l') // right
        {
            angle += 10;
            angle = std::min(k_max_angle, std::max(angle, k_min_angle));
        }

        if(key == 27) break;

        xycar_msgs::xycar_motor motor_message;
        motor_message.speed = speed;
        motor_message.angle = angle;
        publisher.publish(motor_message);

        ros::spinOnce();
    }

    return 0;
}
