// third party header
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

/**
 * @details Test image callback to receive image messages and convert them to cv::Mat
 * @param[in] message Image message accepted by xycar image sensor
 * @return void
 */
void image_callback(const sensor_msgs::Image &message)
{
  cv::Mat image =
      cv::Mat(message.height, message.width, CV_8UC3,
              const_cast<uint8_t *>(&message.data[0]), message.step);
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  cv::imshow("image", image);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, image_callback);

  ros::spin();

  return 0;
}
