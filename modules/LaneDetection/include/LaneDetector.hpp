/**
 * @file LaneDetector.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the LaneDetector class for lane detection and estimation in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__LANEDETECTOR_HPP
#define LANE_DETECTION__LANEDETECTOR_HPP

// Third party header
#include "opencv2/opencv.hpp"

// User defined header
#include "Common.hpp"
#include "KalmanFilter.hpp"

namespace XyCar
{
/**
 * @brief LaneDetector Class for detecting lanes in an image.
 * @details This class is used to detect lanes in an image.
 */
class LaneDetector
{
public:
  using Ptr = LaneDetector *; ///< Pointer type of class.

  /**
   * @brief Construct a new LaneDetector object.
   * @details This function constructs a new LaneDetector object.
   * @param[in] config The configuration of lane_detection project.
   */
  LaneDetector(const YAML::Node &config);

  /**
   * @brief Estimate the lane from the edge image and return the coordinates of the left & right lanes.
   * @details This function estimates the lane from the edge image and return the coordinates of the left & right lanes.
   * @param[in] canny_crop Canny edge ROI image.
   * @param[in] is_refining Flag about whether to refine position of lane.
   * @param[out] draw_image The image for debug.
   * @return State The state of the lanes.
   */
  State find_state(const cv::Mat &canny_crop, cv::Mat &draw_image, bool is_refining = false);

private:
  uint32_t frame_width_;       ///< The frame width of an original image.
  uint32_t roi_frame_y_;        ///< The y-coordinate for roi setting.
  uint32_t offset_;            ///< The offset for position of lane.
  uint32_t lane_width_;        ///< The lane width for estimation.
  int32_t hough_threshold_;    ///< The threshold of hough transition.
  PREC hough_min_line_length_; ///< The min line length of hough transition.
  bool is_right_ = false;      ///< The flag for lane direction.
  cv::Mat canny_crop;          ///< The roi image with canny algorithm.

  State state_;  ///< The state of lane.

  KalmanFilter::Ptr left_kalman_, right_kalman_;  ///< Point class of Kalman filter.

  /**
   * @brief Set values from configuration.
   * @details This function sets values from configuration.
   * @param[in] config The configuration of lane_detection project.
   * @return void
   */
  void set_configuration(const YAML::Node &config);

  /**
   * @brief Divide 'lines' into 'left_lines' and 'right_lines' based on slope.
   * @details This function divides 'lines' into 'left_lines' and 'right_lines' based on slope.
   * @param[in] lines Coordinates consisting of starting and ending points. (x, y)
   * @param[out] left_lines Coordinates of left lines consisting of starting and ending points (x, y).
   * @param[out] right_lines Coordinates of right lines consisting of starting and ending points (x, y).
   * @param[out] stop_lines  Coordinates of stop lines consisting of starting and ending points (x, y).
   * @return void
   */
  void divide_left_right_line(const std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &left_lines,
                              std::vector<cv::Vec4i> &right_lines, std::vector<cv::Vec4i> &stop_lines);

  /**
   * @brief Find the stop line.
   * @details This function finds the stop line.
   * @param[in] stop_lines Coordinates of stop lines consisting of starting and ending points (x, y).
   * @return void
   */
  void find_stop_line(const std::vector<cv::Vec4i> &stoplines);

  /**
   * @brief Calculate the slope and intercept of 'lines', and returns an estimated lane calculated by weighted average.
   * @details This function Calculates the slope and intercept of 'lines', and returns an estimated lane calculated by weighted average.
   * @param[in] lines Coordinates consisting of starting and ending points (x, y).
   * @param[in] is_left Flag of left lane.
   * @return void
   */
  void calculate_slope_and_intercept(const std::vector<cv::Vec4i> &lines, bool is_left = true);

  /**
   * @brief Do exception handling to lane position('pos'), using the 'slope' and 'intercept' of lanes.
   * @details This function does exception handling to lane position('pos'), using the 'slope' and 'intercept' of lanes.
   * @param[in] is_left Flag of left lane.
   * @return void
   */
  void calculate_pos(bool is_left = true);

  /**
   * @brief Estimate left and right lanes based on exception handling.
   * @details This function estimates left and right lanes based on exception handling.
   * @return void
   */
  void refine_pos();

  /**
   * @brief Divide lanes into left & right, and estimate by applying Kalman filter.
   * @details This function divides lanes into left & right, and estimate by applying Kalman filter.
   * @param[in] lines Coordinates consisting of starting and ending points (x, y).
   * @param[out] draw_image The image for debug.
   * @return void
   */
  void evaluate(const std::vector<cv::Vec4i> &lines, const cv::Mat &draw_image);
};
} // namespace XyCar

#endif // LANE_DETECTION__LANEDETECTOR_HPP
