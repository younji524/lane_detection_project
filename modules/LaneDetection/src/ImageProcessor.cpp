/**
 * @file ImageProcessor.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the ImageProcessor class for image pre-processing in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
// User defined header
#include "ImageProcessor.hpp"

namespace XyCar
{
void ImageProcessor::set_configuration(const YAML::Node &config)
{
  roi_frame_y_ = config["IMAGE"]["ROI_Y_POS"].as<uint32_t>();
  frame_width_ = config["IMAGE"]["WIDTH"].as<uint32_t>();
  roi_frame_height_ = config["IMAGE"]["ROI_HEIGHT"].as<uint32_t>();
}

cv::Mat ImageProcessor::process(const cv::Mat &frame)
{
  // crop image
  frame.copyTo(cropped_frame_);
  cropped_frame_ = cropped_frame_(cv::Rect(0, roi_frame_y_, frame_width_, roi_frame_height_));

  // gray image
  cv::cvtColor(cropped_frame_, cropped_frame_, cv::COLOR_BGR2GRAY);

  // blur (gaussian)
  cv::GaussianBlur(cropped_frame_, cropped_frame_, cv::Size(), 2);

  // canny edge
  cv::Canny(cropped_frame_, cropped_frame_, 50, 150);

  return cropped_frame_;
}
} // namespace XyCar
