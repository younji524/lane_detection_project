#include "LaneManager.hpp"

namespace XyCar {
LaneManager::LaneManager() {
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

  publisher_ = node_handler_.advertise<xycar_msgs::xycar_motor>(
      config["TOPIC"]["PUB_NAME"].as<std::string>(),
      config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>());
  subscriber_ =
      node_handler_.subscribe(config["TOPIC"]["SUB_NAME"].as<std::string>(), 1,
                              &LaneManager::image_callback, this);
}

void LaneManager::set_parameters(const YAML::Node &config) {
  frame_width = config["IMAGE"]["WIDTH"].as<uint32_t>();
  frame_height = config["IMAGE"]["HEIGHT"].as<uint32_t>(); // for draw
  offset = config["LANE"]["OFFSET"].as<uint32_t>();        // for draw
  video_name = config["VIDEO"]["VIDEO_NAME"].as<std::string>();
}

void LaneManager::image_callback(const sensor_msgs::Image &message) {
  cv::Mat camera_image =
      cv::Mat(message.height, message.width, CV_8UC3,
              const_cast<uint8_t *>(&message.data[0]), message.step);
  cv::cvtColor(camera_image, image_, cv::COLOR_RGB2BGR);
}

void LaneManager::run() {
  cv::VideoWriter videoWriter;
  std::string video_path;
  node_handler_.getParam("video_path", video_path);

  videoWriter.open(video_path + video_name,
                   cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), k_frame_rate,
                   cv::Size(frame_width, frame_height));

  if (!videoWriter.isOpened()) {
    std::cout << "Can write video !!! check setting" << std::endl;
    return;
  }

  ros::Rate rate(k_frame_rate);
  while (ros::ok()) {
    ros::spinOnce();
    // std::cout << image_.size() << std::endl;

    if (image_.empty())
      continue;

    cv::Mat draw_image = image_.clone();

    cv::Mat canny_image = image_processor_->process(image_);

    State lane_state = detector_->find_state(canny_image, draw_image);

    draw_line_slope(draw_image, lane_state.left_slope_,
                    lane_state.left_intercept_, cv::Scalar(255, 0, 0),
                    frame_height);
    draw_line_slope(draw_image, lane_state.right_slope_,
                    lane_state.right_intercept_, cv::Scalar(255, 0, 0),
                    frame_height);

    // std::cout << "lpos: " << lane_state.left_pos_ << std::endl;
    // std::cout << "rpos: " << lane_state.right_pos_ << std::endl;
    // std::cout << "stop: " << lane_state.stop_flag_ << std::endl;
    cv::putText(draw_image, cv::format("%.1f", lane_state.left_pos_),
                cv::Point(lane_state.left_pos_, offset), cv::FONT_HERSHEY_PLAIN,
                2, cv::Scalar(255, 0, 0));
    cv::putText(draw_image, cv::format("%.1f", lane_state.right_pos_),
                cv::Point(lane_state.right_pos_ - 100, offset),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));

    int32_t lane_centor =
        static_cast<int32_t>(lane_state.right_pos_ + lane_state.left_pos_) >> 1;
    int32_t frame_centor = frame_width >> 1;
    int32_t error = lane_centor - frame_centor;

    draw_rectangle(draw_image, lane_state.left_pos_, cv::Scalar(255, 0, 0),
                   offset);
    draw_rectangle(draw_image, lane_state.right_pos_, cv::Scalar(255, 0, 0),
                   offset);

    draw_rectangle(draw_image, lane_centor, cv::Scalar(255, 0, 0), offset);
    draw_rectangle(draw_image, frame_centor, cv::Scalar(0, 0, 255), offset);

    PREC angle = pid_controller_->compute_angle(error);
    // std::cout <<"error: " << error << std::endl;
    // std::cout <<"angle: " << angle << std::endl;

    publisher_.publish(xycar_controller->control(angle));


    cv::imshow("draw_image", draw_image);
    cv::waitKey(1);
    // cv::imshow("canny_image", canny_image);
    videoWriter << draw_image;
  }
}

} // namespace XyCar
