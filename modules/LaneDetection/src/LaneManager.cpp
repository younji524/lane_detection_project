/**
 * @file LaneManager.cpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the LaneManager class for managing the lane detection system in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
// User defined header
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
 lidar_subscriber_ = node_handler_.subscribe("/scan", 1, &LaneManager::lidar_callback, this);
 object_detect_subscriber_ = node_handler_.subscribe("/yolov3_trt_ros/detections", 1, &LaneManager::object_detection_callback, this);
  yolov3_trt_ros::BoundingBox bbox_list;
}

void LaneManager::set_parameters(const YAML::Node &config)
{
  frame_width_ = config["IMAGE"]["WIDTH"].as<uint32_t>();
  frame_height_ = config["IMAGE"]["HEIGHT"].as<uint32_t>(); // for draw
  offset_ = config["LANE"]["OFFSET"].as<uint32_t>();        // for draw
  video_name_ = config["VIDEO"]["VIDEO_NAME"].as<std::string>();
  wait = config["XYCAR"]["WAIT"].as<int>();
  lidar_angle = config["LIDAR"]["ANGLE"].as<int>();
  lidar_speed = config["LIDAR"]["SPEED"].as<int>();
  lidar_threshold = config["LIDAR"]["THRESHOLD"].as<int>();
  lidar_range = config["LIDAR"]["RANGE"].as<int>();
  detect_x_min = config["OBJECT"]["X_MIN"].as<int>();
  detect_y_min = config["OBJECT"]["Y_MIN"].as<int>();
  detect_x_max = config["OBJECT"]["X_MAX"].as<int>();
  detect_y_max = config["OBJECT"]["Y_MAX"].as<int>();
}

void LaneManager::control_few_second(double time, int angle, int speed)
{
  auto start = std::chrono::steady_clock::now();

  xycar_msgs::xycar_motor motorMessage;

  while(true){
    auto now = std::chrono::steady_clock::now();

    motorMessage.angle = std::round(angle);
    motorMessage.speed = std::round(speed);

    publisher_.publish(motorMessage);

    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);

    if (elapsed.count() >= time) {
        break;
    }
  }
}

void LaneManager::image_callback(const sensor_msgs::Image &message)
{
  cv::Mat camera_image = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t *>(&message.data[0]), message.step);
  cv::cvtColor(camera_image, image_, cv::COLOR_RGB2BGR);
}

void LaneManager::lidar_callback(const sensor_msgs::LaserScan &message)
{
  int is_block[2] = {0,0};

  //count left obstacle
  for (int degree = static_cast<int>((505.0 / 360.0) * 0); degree < static_cast<int>((505.0 / 360.0) * lidar_range); ++degree) {
      if (message.ranges[degree] <= 0.3 && message.ranges[degree] != 0.0) {
        is_block[0]++;
       // std::cout << "left" << "\n";
      }
  }

  //count right obstacle
  for (int degree = static_cast<int>((505.0 / 360.0) * (360 - lidar_range)); degree < static_cast<int>((505.0 / 360.0) * 360); ++degree) {
      if (message.ranges[degree] <= 0.3 && message.ranges[degree] != 0.0) {
        is_block[1]++;
        //std::cout << "right" << "\n";
      }
  }

  if(is_block[0] >= lidar_threshold && is_block[1] >= lidar_threshold)
  {
    lidar_stop();
  }
  else
  {
    if(is_block[0] >= lidar_threshold) // turn right if an obstacle exists left
    {
      lidar_drive(1);
    }
    else if(is_block[1] >= lidar_threshold ) // turn left if an obstacle exists right
    {
      lidar_drive(-1);
    }
  }
}

void LaneManager::lidar_drive(int direction) // turning back after avoiding obstacle by lidar detection
{
  control_few_second(1, lidar_angle * direction, lidar_speed);
  control_few_second(0.5, 0, lidar_speed);
  control_few_second(1, -lidar_angle * direction, lidar_speed);
}

void LaneManager::lidar_stop()
{
  publisher_.publish(xycar_controller->stop());
}

void LaneManager::object_detection_callback(const yolov3_trt_ros::BoundingBoxes &message)
{
  bbox_vector.clear();

  for(yolov3_trt_ros::BoundingBox bbox : message.bounding_boxes)
  {
    std::cout << "x : " << bbox.x << " y : " << bbox.y << "\n";

    if(bbox.id == 0 || bbox.id == 1)
    {
      if(bbox.x < detect_x_max && bbox.x > detect_x_min && bbox.y < detect_y_max && bbox.y > detect_y_min)
      bbox_vector.push_back(bbox);
    }
    else
    {
      if(bbox.x < 150 && bbox.x > 0 && bbox.y < 150 && bbox.y > 0)
      bbox_vector.push_back(bbox);
    }
  }
}

void LaneManager::object_detection_stop() // stop, crosswalk
{
  std::cout << "stop object detect" << "\n";

  control_few_second(wait - 1, 0, 0);
  control_few_second(1, 0, 7);
}

void LaneManager::object_detection_drive(int direction) // left, right
{
  control_few_second(1, 0, 5);
  control_few_second(wait, 50 * direction , 7);
}


void LaneManager::run()
{
  // cv::VideoWriter videoWriter;
  // std::string video_path;
  // node_handler_.getParam("video_path", video_path);

  // videoWriter.open(video_path + video_name_,
  //                  cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), k_frame_rate_,
  //                  cv::Size(frame_width_, frame_height_));

  // if (!videoWriter.isOpened())
  // {
  //   std::cout << "Can write video !!! check setting" << std::endl;
  //   return;
  // }

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
// #if DEBUG
    // std::cout << "lpos: " << lane_state.left_pos_ << std::endl;
    // std::cout << "rpos: " << lane_state.right_pos_ << std::endl;
    // std::cout << "stop: " << lane_state.stop_flag_ << std::endl;
// #endif

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

// #if DEBUG///
    // std::cout <<"error: " << error << std::endl;
    // std::cout <<"angle: " << angle << std::endl;
// #endif

    cv::putText(draw_image, cv::format("%d", error), cv::Point(300, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));
    cv::putText(draw_image, cv::format("%.1f", angle), cv::Point(300, 80), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));


      // if(lane_state.stop_flag_){
      //   publisher_.publish(xycar_controller->stop());
      // }
      // else{
      //   publisher_.publish(xycar_controller->control(angle));
      // }
    if(bbox_vector.empty())
    {
      publisher_.publish(xycar_controller->control(angle));
    }
    else
    {
      if (lane_state.stop_flag_)
      {
        std::cout << "ID : " << bbox_vector[0].id << "\n";
        if(bbox_vector[0].id == 2 || bbox_vector[0].id == 3) // stop(2), crossewalk(3)
          object_detection_stop();
        else if(bbox_vector[0].id == 4) // traffic_light_red(4)
          publisher_.publish(xycar_controller->stop());
        else if(bbox_vector[0].id == 0) // left(0)
        {
          std::cout << "stop" << "\n";
          control_few_second(1, 0, 0);
          object_detection_drive(-1);
        }
        else if(bbox_vector[0].id == 1) // right(1)
        {
          std::cout << "stop" << "\n";
          control_few_second(1, 0, 0);
          object_detection_drive(1);
        }
        else // when xycar, traffic_light_green, ignore detected
          publisher_.publish(xycar_controller->control(angle));
      }
      else
      {
        if(bbox_vector[0].id == 0)
          object_detection_drive(-1);
        else if(bbox_vector[0].id == 1)
          object_detection_drive(1);
      }
    }

    cv::imshow("draw_image", draw_image);
    cv::waitKey(1);
    // videoWriter << draw_image;
  }
}

} // namespace XyCar
