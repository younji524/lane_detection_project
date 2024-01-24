/**
 * @file LaneManager.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the LaneManager class for managing the lane detection system in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__LANEMANAGER_HPP
#define LANE_DETECTION__LANEMANAGER_HPP

// System header
#include <iostream>
#include <string>
#include <vector>
// Third party header
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "yolov3_trt_ros/BoundingBoxes.h"
#include "yolov3_trt_ros/BoundingBox.h"
// User defined header
#include "Common.hpp"
#include "draw.hpp"
#include "ImageProcessor.hpp"
#include "LaneDetector.hpp"
#include "PIDController.hpp"
#include "XycarController.hpp"

namespace XyCar
{
/**
 * @brief LaneManager Class that manage lane detection system.
 * @details This class is used to manage lane detection system.
 */
class LaneManager
{
public:
  /**
   * @brief Construct a new LaneManager object.
   * @details This function construct a new LaneManager object.
   */
  LaneManager();

  /**
   * @brief Run from image processing to publishing motor control topics.
   * @details This function run from image processing to publishing motor control topics.
   * @return void
   */
  void run();

private:
  ros::NodeHandle node_handler_; ///< A Node handler for detector and controller.
  ros::Subscriber subscriber_; ///< A subscriber for reading images.
  ros::Publisher publisher_; ///< A publisher for motor control.
  ros::Subscriber lidar_subscriber_; //< A subscriber for lidar data.
  ros::Subscriber object_detect_subscriber_; //< A subscriber for object data.

  ImageProcessor::Ptr image_processor_; ///< Image processor class for image processing.
  LaneDetector::Ptr detector_; ///< Lane detector class for detecting lane.
  PIDController::Ptr pid_controller_; ///< PID class for control.
  XycarController::Ptr xycar_controller; ///< Xycar controller class for motor control.
  std::vector<yolov3_trt_ros::BoundingBox> bbox_vector;

  // TODO: 큐 사용해보기
  // std::queue <cv::Mat> current_images_;
  cv::Mat image_; ///< An original image from usb cam.
  static constexpr double k_frame_rate_ = 33.0; ///< Frame rate.
  uint32_t frame_width_;  ///< The frame width of original image.
  uint32_t frame_height_; ///< The frame height for draw.
  uint32_t offset_;       ///< The offset for draw.
  std::string video_name_;  ///< The output video name.
  int lidar_angle;
  int lidar_speed;
  int lidar_threshold;
  int lidar_range;
  int detect_x_min;
  int detect_y_min;
  int detect_x_max;
  int detect_y_max;
  int wait;

  /**
   * @brief Set configuration.
   * @details This function sets configuration.
   * @param[in] config The configuration of lane_detection project.
   * @return void
   */
  void set_parameters(const YAML::Node &config);

  /**
   * @brief Convert sensor_msgs::Image to cv::Mat and change to grayscale.
   * @details This function converts sensor_msgs::Image to cv::Mat and change to grayscale.
   * @param[in] message Image topic message.
   * @return void
   */
  void image_callback(const sensor_msgs::Image &message);
  void lidar_callback(const sensor_msgs::LaserScan &message);
  void object_detection_callback(const yolov3_trt_ros::BoundingBoxes &message);
  void lidar_drive(int direction); // lidar 객체 인식 후 지정된 방향으로 이동 후 복귀
  void lidar_stop(); // lidar 객체 인식 후 정지
  void object_detection_stop(); // stop, crosswalk 인식 -> 잠시 정지 후 이동
  void object_detection_drive(int direction); // left, right 인식
  void object_detection_green_light(); // 초록 신호 인식
  void object_detection_red_light(); // 빨간 신호 인식
  void control_few_second(double time, int angle, int speed);
};
} // namespace XyCar

#endif // LANE_DETECTION__LANEMANAGER_HPP