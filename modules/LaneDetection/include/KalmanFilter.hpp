/**
 * @file KalmanFilter.hpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Defines the KalmanFilter class for state estimation in the XyCar namespace.
 * @version 1.0.0
 * @date 2023-11-10
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
#ifndef LANE_DETECTION__KALMANFILTER_HPP
#define LANE_DETECTION__KALMANFILTER_HPP

// Third party header
#include "opencv2/opencv.hpp"
// User defined header
#include "Common.hpp"

namespace XyCar
{
/**
 * @brief KalmanFilter class.
 * @details This class is used to apply Kalman filter for state estimation.
 */
class KalmanFilter
{
public:
  using Ptr = KalmanFilter *;  ///< Pointer type of class.

  /**
   * @brief Construct a new KalmanFilter object.
   * @details This function constructs a new KalmanFilter object.
   * @param[in] config The configuration of lane_detection project.
   */
  KalmanFilter(const YAML::Node &config);

  /**
   * @brief Perform Kalman filtering.
   * @details This function perform Kalman filtering.
   * Using the is_first_ flag, the initial state receives detected data.
   * @param slope The slope of lane.
   * @param intercept The intercept of lane.
   * @return void
   */
  void kalman_filtering(PREC slope, PREC intercept);

  /**
   * @brief Get the state matrix.
   * @details This function gets the state matrix.
   * When you want to receive the slope and intercept, you receive it using the cv::Mat::at() method.
   * @code{.cpp}
   * state_mat = kalman.get_state();
   * slope = state_mat.at<PREC>(0, 0);
   * intercept = state_mat.at<PREC>(2, 0);
   * @endcode
   * @return const Mat_<PREC>& (Here, PREC refers to double) The filtered state.
   */
  const cv::Mat_<PREC> &get_state() const;

private:
  PREC slope_derivative_;     ///< The differential term of slope of lane.
  PREC intercept_derivative_; ///< The differential term of intercept of lane.
  PREC dt_;                   ///< The time difference between measurements.
  bool is_first_ = true;      ///< The flag that indicates if it's first measurement.

  cv::Mat_<PREC> state_matrix_;             ///< x: The State estimates (상태 추정치)
  cv::Mat_<PREC> transition_matrix_;        ///< A: The State Transition Matrix (상태 변환 행렬)
  cv::Mat_<PREC> transition_matrix_t_;      ///< A^T: The transpose matrix of State Transition Matrix
  cv::Mat_<PREC> measurement_matrix_;       ///< H: The Measurement Matrix (측정 행렬)
  cv::Mat_<PREC> measurement_matrix_t_;     ///< H^T: The transpose matrix of Measurement Matrix
  cv::Mat_<PREC> process_noise_matrix_;     ///< Q: The Process Noise Covariance Matrix (과정 노이즈 공분산)
  cv::Mat_<PREC> measurement_noise_matrix_; ///< R: The Measurement Noise Covariance Matrix (측정 노이즈 공분산)
  cv::Mat_<PREC> covariance_matrix_;        ///< P: The Error Covariance matrix (오차 공분산)
  cv::Mat_<PREC> kalman_gain_;              ///< K: The Kalman gain (칼만 이득)

  /**
   * @brief Performs the prediction steps of the Kalman filter.
   * @details This fuction performs the prediction steps of the Kalman filter.
   * Predicts the state vector using a given 'slope' and 'intercept'.
   * Then, calculate the predicted covariance using the transition matrix and the covariance matrix.
   * @param[in] slope The slope of lane.
   * @param[in] intercept The intercept of lane.
   * @return void
   */
  void predict(PREC slope, PREC intercept);

  /**
   * @brief Update the state of a Kalman filter.
   * @details This function updates the internal state of a Kalman filter given the 'slope', 'intercept'.
   * The Kalman gain is computed, and the state matrix and covariance matrix are updated accordingly.
   * If the slope is not equal to 0, or the intercept is not equal to 0,
   * the state matrix is updated with a measurement derived from the slope and intercept.
   * @param[in] slope The slope of the lane.
   * @param[in] intercept The intercept of the lane.
   * @return void
   */
  void update(PREC slope, PREC intercept);
};
} // namespace XyCar

#endif // LANE_DETECTION__KALMANFILTER_HPP
