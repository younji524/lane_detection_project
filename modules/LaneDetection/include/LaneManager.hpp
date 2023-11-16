#ifndef LANE_DETECTION__LANEMANAGER_HPP
#define LANE_DETECTION__LANEMANAGER_HPP

// system header
#include <iostream>
#include <string>

// third party header
#include "sensor_msgs/Image.h"
// user defined header
#include "Common.hpp"
#include "ImageProcessor.hpp"
#include "LaneDetector.hpp"
#include "PIDController.hpp"
#include "XycarController.hpp"
#include "draw.hpp"

namespace XyCar {
/**
 * @details Class that manage lane detection system.
 */
class LaneManager {
public:
  /**
   * @details Construct a new Lane Manager object
   */
  LaneManager();

  /**
   * @details Run from image processing to publishing motor control topics.
   * @return void
   */
  void run();

private:
  ros::NodeHandle
      node_handler_;           ///< A Node handler for detector and controller.
  ros::Subscriber subscriber_; ///< A subscriber for reading images.
  ros::Publisher publisher_;   ///< A publisher for motor control.

  ImageProcessor::Ptr
      image_processor_;        ///< Image processor class for image processing.
  LaneDetector::Ptr detector_; ///< Lane detector class for detecting lane.
  PIDController::Ptr pid_controller_; ///< PID class for control.
  XycarController::Ptr
      xycar_controller; ///< Xycar controller class for motor control.

  // TODO: 큐 사용해보기
  // std::queue <cv::Mat> current_images_;
  cv::Mat image_; ///< An original image from usb cam.
  static constexpr double k_frame_rate = 33.0; ///< Frame rate
  uint32_t frame_width;  ///< The frame width of original image.
  uint32_t frame_height; ///< The frame height for draw.
  uint32_t offset;       ///< The offset for draw.
  std::string video_name;

  /**
   * @details  Set configuration.
   * @param[in]  config  The configuration of lane_detection project.
   */
  void set_parameters(const YAML::Node &config);

  /**
   * @details Convert sensor_msgs::Image to cv::Mat and change to grayscale.
   * @param[in] message Image topic message.
   * @return void
   */
  void image_callback(const sensor_msgs::Image &message);
};
} // namespace XyCar

#endif // LANE_DETECTION__LANEMANAGER_HPP