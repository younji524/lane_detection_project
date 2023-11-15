#ifndef LANE_DETECTION__KALMANFILTER_HPP
#define LANE_DETECTION__KALMANFILTER_HPP

// Third party header
#include "opencv2/opencv.hpp"
// User defined header
#include "Common.hpp"

namespace XyCar
{
/**
 * @details  Class of the Kalman filter.
 */
class KalmanFilter
{
public:
    using Ptr = KalmanFilter*;
    KalmanFilter(const YAML::Node& config);

    /**
     * @details Initialize objects for applying the Kalman filter.
     * @return void
     */
    // void init();

    void kalman_filtering(PREC slope, PREC intercept);

    /**
     * @details Get the state matrix. \n
     * When you want to receive the slope and intercept, \n
     * you receive it using the cv::Mat::at() method. \n
     * ex) slope = state_mat.at<PREC>(0, 0); \n
     * ex) intercept = state_mat.at<PREC>(2, 0); \n
     * @return const Mat&
     */
    const cv::Mat_<PREC>& get_state() const;

private:
    PREC slope_derivative_; ///< The differential term of slope of lane.
    PREC intercept_derivative_;  ///< The differential term of intercept of lane.
    PREC dt_;
    bool is_first_ = true;

    cv::Mat_<PREC> state_matrix_;  ///< x: The State estimates (상태 추정치)
    cv::Mat_<PREC> transition_matrix_;  ///< A: The State Transistion Matrix (상태 변환 행렬)
    cv::Mat_<PREC> measurement_matrix_;  ///< H: The Measurement Matrix (측정 행렬)
    cv::Mat_<PREC> process_noise_matrix_;  ///< Q: The Process Noise Covariance Matrix (과정 노이즈 공분산)
    cv::Mat_<PREC> measurement_noise_matrix_;  ///< R: The Measurement Noise Covariance Matrix (측정 노이즈 공분산)
    cv::Mat_<PREC> covariance_matrix_;  ///< P: The Error Covariance matrix (오차 공분산)
    cv::Mat_<PREC> kalman_gain_;  ///< K: The Kalman gain (칼만 이득)


  /**
   * @details Perform the prediction steps of the Kalman filter. \n
   *          Predicts the state vector using a given 'slope' and 'intercept'.
   * \n Then, calculate the predicted covariance using the transition matrix and
   * the covariance matrix. \n
   * @param[in] slope The slope of lane.
   * @param[in] intercept The intercept of lane.
   * @return void
   */
  void predict(PREC slope, PREC intercept);

  /**
   * @details Update the internal state of a Kalman filter given the 'average
   * slope', 'average intercept' and 'position of lane'. \n The Kalman gain is
   * computed, and the state matrix and covariance matrix are updated
   * accordingly. \n If the position is less than or equal to 0, or greater than
   * or equal to 640, the state matrix is updated with a measurement matrix
   * derived from the average slope and intercept. \n
   * @param[in] avg_slope The average slope of the lane.
   * @param[in] avg_intercept The intercept slope of the lane.
   * @param[in] pos The position of the lane. (x coordinate)
   * @return void
   */
  void update(PREC avg_slope, PREC avg_intercept, PREC pos);

  /**
   * @details Get the state matrix. \n
   * When you want to receive the slope and intercept, \n
   * you receive it using the cv::Mat::at() method. \n
   * ex) slope = state_mat.at<PREC>(0, 0); \n
   * ex) intercept = state_mat.at<PREC>(2, 0); \n
   * @return const Mat&
   */


};
} // XyCar

#endif // LANE_DETECTION__KALMANFILTER_HPP
