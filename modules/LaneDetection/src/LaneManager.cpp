/**
 * @file LaneManager.cpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the LaneManager class for managing the lane detection system in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#include "LaneManager.hpp"

namespace XyCar
{
LaneManager::LaneManager()
{
  std::string configPath;
  node_handler_.getParam("config_path", configPath);
  YAML::Node config = YAML::LoadFile(configPath);

  pid_controller_ = new PIDController(config["PID"]["P_GAIN"].as<PREC>(),
                                      config["PID"]["I_GAIN"].as<PREC>(),
                                      config["PID"]["D_GAIN"].as<PREC>());
  image_processor_ = new ImageProcessor(config);
  detector_ = new LaneDetector(config);
  xycar_controller = new XycarController(config);

  set_parameters(config);

  publisher_ = node_handler_.advertise<xycar_msgs::xycar_motor>(config["TOPIC"]["PUB_NAME"].as<std::string>(), config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>());
  subscriber_ = node_handler_.subscribe(config["TOPIC"]["SUB_NAME"].as<std::string>(), 1,
                              &LaneManager::image_callback, this);
}

void LaneManager::set_parameters(const YAML::Node &config)
{
  frame_width_ = config["IMAGE"]["WIDTH"].as<uint32_t>();
  frame_height_ = config["IMAGE"]["HEIGHT"].as<uint32_t>(); // for draw
  offset_ = config["LANE"]["OFFSET"].as<uint32_t>();        // for draw
  video_name_ = config["VIDEO"]["VIDEO_NAME"].as<std::string>();
}

void LaneManager::image_callback(const sensor_msgs::Image &message)
{
  cv::Mat camera_image = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t *>(&message.data[0]), message.step);
  cv::cvtColor(camera_image, image_, cv::COLOR_RGB2BGR);
}

void LaneManager::run()
{
  cv::VideoWriter videoWriter;
  std::string video_path;
  node_handler_.getParam("video_path", video_path);

  videoWriter.open(video_path + video_name_,
                   cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), k_frame_rate_,
                   cv::Size(frame_width_, frame_height_));

  if (!videoWriter.isOpened())
  {
    std::cout << "Can write video !!! check setting" << std::endl;
    return;
  }

  ros::Rate rate(k_frame_rate_);
  while (ros::ok())
  {
    ros::spinOnce();

    if (image_.empty())
      continue;

    cv::Mat draw_image = image_.clone();

    cv::Mat canny_image = image_processor_->process(image_);

    State lane_state = detector_->find_state(canny_image, draw_image);

    draw_line_slope(draw_image, lane_state.left_slope_,
                    lane_state.left_intercept_, cv::Scalar(255, 0, 0),
                    frame_height_);
    draw_line_slope(draw_image, lane_state.right_slope_,
                    lane_state.right_intercept_, cv::Scalar(255, 0, 0),
                    frame_height_);
#if DEBUG
    std::cout << "lpos: " << lane_state.left_pos_ << std::endl;
    std::cout << "rpos: " << lane_state.right_pos_ << std::endl;
    std::cout << "stop: " << lane_state.stop_flag_ << std::endl;
#endif

    cv::putText(draw_image, cv::format("%.1f", lane_state.left_pos_),
                cv::Point(lane_state.left_pos_, offset_), cv::FONT_HERSHEY_PLAIN,
                2, cv::Scalar(255, 0, 0));
    cv::putText(draw_image, cv::format("%.1f", lane_state.right_pos_),
                cv::Point(lane_state.right_pos_ - 100, offset_),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));

    int32_t lane_centor = static_cast<int32_t>(lane_state.right_pos_ + lane_state.left_pos_) >> 1;
    int32_t frame_centor = frame_width_ >> 1;
    int32_t error = lane_centor - frame_centor;

    draw_rectangle(draw_image, lane_state.left_pos_, cv::Scalar(255, 0, 0), offset_);
    draw_rectangle(draw_image, lane_state.right_pos_, cv::Scalar(255, 0, 0), offset_);

    draw_rectangle(draw_image, lane_centor, cv::Scalar(255, 0, 0), offset_);
    draw_rectangle(draw_image, frame_centor, cv::Scalar(0, 0, 255), offset_);

    PREC angle = pid_controller_->compute_angle(error);

#if DEBUG
    std::cout <<"error: " << error << std::endl;
    std::cout <<"angle: " << angle << std::endl;
#endif

    cv::putText(draw_image, cv::format("%d", error), cv::Point(300, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));
    cv::putText(draw_image, cv::format("%.1f", angle), cv::Point(300, 80), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));

    publisher_.publish(xycar_controller->control(angle));

    cv::imshow("draw_image", draw_image);
    cv::waitKey(1);
    videoWriter << draw_image;
  }
}

} // namespace XyCar
