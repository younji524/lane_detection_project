#include "KalmanFilter.hpp"

namespace XyCar
{
    KalmanFilter::KalmanFilter()
    {
        dt_ = 1.0 / 30;
        slope_derivative_ = -0.0005;
        intercept_derivative_ = -0.1;

        transition_matrix_ = (cv::Mat_<PREC>(4, 4) << 1, dt_, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt_, 0, 0, 0, 1);
        measurement_matrix_ = (cv::Mat_<PREC>(2,4) << 1, 0, 0, 0, 0, 0, 1, 0);
        process_noise_matrix_ = cv::Mat::eye(4, 4, CV_64F);
        measurement_noise_matrix_ = (cv::Mat_<PREC>(2, 2) << 50, 0, 0, 50);
        covariance_matrix_ = 100 * cv::Mat::eye(4, 4, CV_64F);
    }

    void KalmanFilter::kalman_filtering(PREC slope, PREC intercept)
    {
        if(is_first_){
            estimation_slope_ = slope;
            estimation_intercept_ = intercept;
            is_first_ = false;
        }
        
        predict(estimation_slope_, estimation_intercept_);
        update(slope, intercept);
    }

    void KalmanFilter::predict(PREC slope, PREC intercept)
    {
        state_matrix_ = transition_matrix_ * (cv::Mat_<PREC>(4,1) << slope, slope_derivative_, intercept, intercept_derivative_);
        static const auto transition_matrix_t = transition_matrix_.t();
        covariance_matrix_ = transition_matrix_ * covariance_matrix_ * transition_matrix_t + process_noise_matrix_;
    }

    void KalmanFilter::update(PREC avg_slope, PREC avg_intercept)
    {
        // kalman gain
        static const auto measurement_matrix_t = measurement_matrix_.t();
        kalman_gain_ = covariance_matrix_ * measurement_matrix_t * (measurement_matrix_ * covariance_matrix_ * measurement_matrix_t + measurement_noise_matrix_).inv();

        // if (pos > 0 && pos < 640)
        if(avg_slope != 0 || avg_intercept != 0)
        {
            cv::Mat measurement = (cv::Mat_<PREC>(2, 1) << avg_slope, avg_intercept);
            state_matrix_ += kalman_gain_ * (measurement - measurement_matrix_ * state_matrix_);
        }

        // Update covariance
        covariance_matrix_ -= kalman_gain_ * measurement_matrix_ * covariance_matrix_;
    }

    const cv::Mat_<PREC>& KalmanFilter::get_state() const
    {
        return state_matrix_;
    }
} // XyCar
