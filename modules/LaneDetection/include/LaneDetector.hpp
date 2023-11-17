//
// Created by nahye on 23. 11. 7.
//

#ifndef LANE_DETECTION__LANEDETECTOR_HPP
#define LANE_DETECTION__LANEDETECTOR_HPP

// Third party header
#include "opencv2/opencv.hpp"
#include <iostream>
// User defined header
#include "Common.hpp"
#include "KalmanFilter.hpp"

namespace XyCar {
/**
 * @details  Class responsible for detecting lanes in an image.
 */
class LaneDetector {
public:
  using Ptr = LaneDetector *; ///< Pointer type of class.
  cv::Mat canny_crop;         ///< The roi image with canny algorithm.

  /**
   * @details Construct a new Lane Detector object
   * @param[in] config The configuration of lane_detection project.
   */
  LaneDetector(const YAML::Node &config) { set_configuration(config); }

  /**
   * @details Estimate the lane from the edge image and return the coordinates
   * of the left & right lanes.
   * @param[in] canny_crop Canny edge ROI image.
   * @param[in] is_refining Flag about whether to refine position of lane.
   * @return State
   */
  // State find_state(const cv::Mat& canny_crop, bool is_refining = false)
  State find_state(const cv::Mat &canny_crop, cv::Mat &draw_image,
                   bool is_refining = false) {
    std::vector<cv::Vec4i> lines;
    constexpr PREC rho = 1.0;
    constexpr PREC theta = CV_PI / 180.0;
    constexpr PREC min_line_gap = 5;

    cv::HoughLinesP(canny_crop, lines, rho, theta, hough_threshold, hough_min_line_length, min_line_gap);

    cv::Mat hough_image = canny_crop.clone();
    cv::cvtColor(hough_image ,hough_image, cv::COLOR_GRAY2BGR);
    for(cv::Vec4i line : lines) {
        cv::line(hough_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255,0,255), 2, cv::LINE_8);
    }
    cv::imshow("hough_image", hough_image);

    evaluate(lines, draw_image);

    if (is_refining)
      refine_pos();

    return state_;
  }

private:
  uint32_t frame_width; ///< The frame width of an original image.
  uint32_t roi_frame_y; ///< The y-coordinate for roi setting.
  uint32_t offset;      ///< The offset for position of lane.
  uint32_t lane_width;  ///< The lane width for estimation.
  bool is_right = false;
  int32_t hough_threshold; ///< The threshole of hough transition.
  PREC hough_min_line_length; ///< The min line length of hough transition.

  State state_; ///< The state of lane.

  KalmanFilter::Ptr left_kalman_, right_kalman_;

  /**
   * @details Set values from configuration.
   * @param[in] config The configuration of lane_detection project.
   * @return void
   */
  void set_configuration(const YAML::Node &config);

  /**
   * @details Divide 'lines' into 'left_lines' and 'right_lines' based on slope.
   * @param[in] lines Coordinates consisting of starting and ending points. (x,
   * y)
   * @param[out] left_lines Coordinates of left lines consisting of starting and
   * ending points (x, y).
   * @param[out] right_lines Coordinates of right lines consisting of starting
   * and ending points (x, y).
   * @param[out] stop_lines  Coordinates of stop lines consisting of starting
   * and ending points (x, y).
   * @return void
   */
  void divide_left_right_line(const std::vector<cv::Vec4i> &lines,
                              std::vector<cv::Vec4i> &left_lines,
                              std::vector<cv::Vec4i> &right_lines,
                              std::vector<cv::Vec4i> &stop_lines);

  /**
   * @details Find the stop line.
   * @param[in] stop_lines Coordinates of stop lines consisting of starting
   * and ending points (x, y).
   * @return void
   */
  void find_stop_line(const std::vector<cv::Vec4i> &stoplines);

  /**
   * @details Calculates the slope and intercept of 'lines',
   * and returns an estimated lane calculated by weighted average.
   * @param[in] lines Coordinates consisting of starting and ending points (x,
   * y).
   * @param[in] is_left Flag of left lane.
   * @return void
   */
  void calculate_slope_and_intercept(const std::vector<cv::Vec4i> &lines,
                                     bool is_left = true);

  /**
   * @details Do exception handling to lane position('pos'),
   * using the 'slope' and 'intercept' of lanes.
   * @param[in] is_left Flag of left lane.
   * @return void
   */
  void calculate_pos(bool is_left = true);

  /**
   * @details Estimate left and right lanes based on exception handling.
   * @return void
   */
  void refine_pos();

  /**
   * @details Divide lanes into left & right,
   * and estimate by applying filter (or our algorithm).
   * @param[in] lines Coordinates consisting of starting and ending points (x,
   * y).
   * @return void
   */
  // void evaluate(const std::vector<cv::Vec4i>& lines)
  void evaluate(const std::vector<cv::Vec4i> &lines,
                const cv::Mat &draw_image) {
    std::vector<cv::Vec4i> left_lines, right_lines, stop_lines;
    divide_left_right_line(lines, left_lines, right_lines, stop_lines);

    for (cv::Vec4i line : left_lines) {
      cv::line(draw_image, cv::Point(line[0], line[1] + roi_frame_y),
               cv::Point(line[2], line[3] + roi_frame_y),
               cv::Scalar(255, 0, 255), 2, cv::LINE_8);
    }
    for (cv::Vec4i line : right_lines) {
      cv::line(draw_image, cv::Point(line[0], line[1] + roi_frame_y),
               cv::Point(line[2], line[3] + roi_frame_y),
               cv::Scalar(255, 255, 0), 2, cv::LINE_8);
    }

    calculate_slope_and_intercept(left_lines);
    calculate_slope_and_intercept(right_lines, is_right);

    cv::Mat_<PREC> kalman_estimation_matrix;

    left_kalman_->kalman_filtering(state_.left_slope_, state_.left_intercept_);
    kalman_estimation_matrix = left_kalman_->get_state();
    state_.left_slope_ = kalman_estimation_matrix.at<PREC>(0, 0);
    state_.left_intercept_ = kalman_estimation_matrix.at<PREC>(2, 0);

    right_kalman_->kalman_filtering(state_.right_slope_,
                                    state_.right_intercept_);
    kalman_estimation_matrix = right_kalman_->get_state();
    state_.right_slope_ = kalman_estimation_matrix.at<PREC>(0, 0);
    state_.right_intercept_ = kalman_estimation_matrix.at<PREC>(2, 0);

    // find_stop_line(stop_lines);

    // calculate lpos, rpos
    calculate_pos();
    calculate_pos(is_right);
  }
};
} // namespace XyCar

#endif // LANE_DETECTION__LANEDETECTOR_HPP
