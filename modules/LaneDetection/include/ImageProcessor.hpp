#ifndef LANE_DETECTION__IMAGEPROCESSOR_HPP
#define LANE_DETECTION__IMAGEPROCESSOR_HPP

// Third party header
#include "opencv2/opencv.hpp"

// User defined header
#include "Common.hpp"

namespace XyCar
{
/**
 * @details Class responsible for pre-processing of the image for lane detection.
 */
class ImageProcessor
{
public:
  using Ptr = ImageProcessor *; ///< Pointer type of class.

  /**
   * @details  Construct a new Image Processor object
   * @param[in]  config  The configuration of lane_detection project.
   */
  ImageProcessor(const YAML::Node &config) { set_configuration(config); }

  /**
   * @details  Perform the image preprocessing tasks below.
   * 1. Crop the image for ROI settings.
   * 2. Convert to grayscale.
   * 3. Perform a Gaussian Blur.
   * 4. Apply a Canny Algorithm.
   * @param[in]  frame  The original video frame.
   * @return  cv::Mat
   */
  cv::Mat process(const cv::Mat &frame);

private:
  cv::Mat cropped_frame_;    ///< The cropped image frame for lane detection.
  uint32_t roi_frame_y_;      ///< The y-coordinate for roi setting.
  uint32_t frame_width_;      ///< The frame width of an original image.
  uint32_t roi_frame_height_; ///< The height of frame for roi setting.

  /**
   * @details  Set values from configuration.
   * @param[in]  config  The configuration of lane_detection project.
   * @return  void
   */
  void set_configuration(const YAML::Node &config);
};
} // namespace XyCar

#endif // LANE_DETECTION__IMAGEPROCESSOR_HPP
