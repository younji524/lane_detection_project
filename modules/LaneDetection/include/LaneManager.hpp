#ifndef LANE_DETECTION__LANEMANAGER_HPP
#define LANE_DETECTION__LANEMANAGER_HPP

// system header
#include <queue>
#include <iostream>

// third party header
#include "sensor_msgs/Image.h"

// user defined header
#include "Common.hpp"
#include "ImageProcessor.hpp"
#include "LaneDetector.hpp"
#include "PIDController.hpp"
#include "XycarController.hpp"

namespace XyCar
{
class LaneManager
{
public:
    LaneManager();
    void run();

private:
    ros::NodeHandle node_handler_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;

    ImageProcessor::Ptr image_processor_;
    LaneDetector::Ptr detector_;
    PIDController::Ptr pid_controller_;
    XycarController::Ptr  xycar_controller;

    // std::queue <cv::Mat> current_images_;
    cv::Mat image_;
    static constexpr double k_frame_rate = 33.0; ///< Frame rate
    uint32_t frame_width;
    uint32_t frame_height; //for draw
    uint32_t offset; //for draw

    void set_parameters(const YAML::Node& config);
    void image_callback(const sensor_msgs::Image& message);
};
} // XyCar

#endif // LANE_DETECTION__LANEMANAGER_HPP