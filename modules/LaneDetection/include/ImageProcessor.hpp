#ifndef LANE_DETECTION__IMAGEPROCESSOR_HPP
#define LANE_DETECTION__IMAGEPROCESSOR_HPP

// System header
#include <cstdint>
// Third party header
#include "opencv2/opencv.hpp"
// User defined header
#include "Common.hpp"

namespace XyCar {
/**
 * @details Class responsible for pre-processing of the image for lane
 * detection.
 */
class ImageProcessor {
public:
  using Ptr = ImageProcessor *; ///< Pointer type of class.

  /**
   * @details  Construct a new Image Processor object
   * @param[in]  config  The configuration of lane_detection project.
   */
  ImageProcessor(const YAML::Node &config) { set_configuration(config); }

  /**
   * @details  Perform the image preprocessing tasks below.\n
   * 1. Crop the image for ROI settings.\n 2. Convert to grayscale.\n
   * 3. Perform Histogram Equalization & Binarization.\n 4. Apply the LiDAR
   * mask.\n
   * 5. Perform a Gaussian Blur.\n 6. Apply a Canny Algorithm.\n
   * @param[in]  frame  The original video frame.
   * @return  cv::Mat
   */
  cv::Mat process(const cv::Mat &frame);

private:
  cv::Mat cropped_frame_;    ///< The cropped image frame for lane detection.
  uint32_t roi_frame_y;      ///< The y-coordinate for roi setting.
  uint32_t frame_width;      ///< The frame width of an original image.
  uint32_t roi_frame_height; ///< The height of frame for roi setting.

  /**
   * @details  Set values from configuration.
   * @param[in]  config  The configuration of lane_detection project.
   * @return  void
   */
  void set_configuration(const YAML::Node &config);
};
} // namespace XyCar

#endif // LANE_DETECTION__IMAGEPROCESSOR_HPP
