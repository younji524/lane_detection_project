/**
 * @file ImageProcessor.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the ImageProcessor class for image pre-processing in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__IMAGEPROCESSOR_HPP
#define LANE_DETECTION__IMAGEPROCESSOR_HPP

// Third party header
#include "opencv2/opencv.hpp"
// User defined header
#include "Common.hpp"

namespace XyCar
{
/**
 * @brief ImageProcessor class for pre-processing of the image.
 * @details This class is used to process pre-processing of the image for lane detection.
 */
class ImageProcessor
{
public:
  using Ptr = ImageProcessor *; ///< Pointer type of class.

  /**
   * @brief Construct a new ImageProcessor object.
   * @details This function constructs a new ImageProcessor object.
   * @param[in] config The configuration of lane_detection project.
   */
  ImageProcessor(const YAML::Node &config) { set_configuration(config); }

  /**
   * @brief Perform the image preprocessing tasks.
   * @details This function performs the image preprocessing tasks below.
   * 1. Crop the image for ROI settings.
   * 2. Convert to grayscale.
   * 3. Perform a Gaussian Blur.
   * 4. Apply a Canny Algorithm.
   * @param[in] frame The original video frame.
   * @return cv::Mat Pre-processed image.
   */
  cv::Mat process(const cv::Mat &frame);

private:
  cv::Mat cropped_frame_;     ///< The cropped image frame for lane detection.
  uint32_t roi_frame_y_;      ///< The y-coordinate for roi setting.
  uint32_t frame_width_;      ///< The frame width of an original image.
  uint32_t roi_frame_height_; ///< The height of frame for roi setting.

  /**
   * @brief Set values from configuration.
   * @details This function sets values from configuration.
   * @param[in] config The configuration of lane_detection project.
   * @return void
   */
  void set_configuration(const YAML::Node &config);
};
} // namespace XyCar

#endif // LANE_DETECTION__IMAGEPROCESSOR_HPP
