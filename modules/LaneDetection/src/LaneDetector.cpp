//
// Created by nahye on 23. 11. 7.
//

#include "LaneDetector.hpp"

namespace XyCar
{
    void LaneDetector::set_configuration(const YAML::Node& config)
    {
        roi_frame_y = config["IMAGE"]["ROI_Y_POS"].as<uint32_t>();
        frame_width = config["IMAGE"]["WIDTH"].as<uint32_t>();
        offset = config["LANE"]["OFFSET"].as<uint32_t>();
        lane_width = config["LANE"]["LANE_WIDTH"].as<uint32_t>();

        left_kalman_ = new KalmanFilter(config);
        right_kalman_ = new KalmanFilter(config);

    }

    void LaneDetector::divide_left_right_line(const std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& left_lines, std::vector<cv::Vec4i>& right_lines, std::vector<cv::Vec4i>& stop_lines)
    {
        constexpr double k_low_slope_threshold = 0.1;
        constexpr double k_stop_slpoe_threshold = 0.15;

        // int32_t half_frame = frame_width / 2;
        int32_t threshold_location = frame_width / 5;

        int32_t left_range = frame_width * 0.6;
        int32_t right_range = frame_width * 0.4;


        for(const cv::Vec4i& line : lines)
        {
            int32_t x1 = line[0];
            int32_t y1 = line[1];
            int32_t x2 = line[2];
            int32_t y2 = line[3];

            if(x2 == x1)
                continue;

            double slope = static_cast<double>(y2 - y1) / (x2 - x1);

            if((slope < -k_low_slope_threshold) && (x1 < left_range))
                left_lines.emplace_back(x1,y1,x2,y2);

            else if((slope > k_low_slope_threshold) && (x2 > right_range))
                right_lines.emplace_back(x1,y1,x2,y2);

            else if((abs(slope) <= k_stop_slpoe_threshold) && (x1 > threshold_location) && (x2 < threshold_location * 4))
                stop_lines.emplace_back(x1,y1,x2,y2);
        }
    }

    void LaneDetector::find_stop_line(const std::vector<cv::Vec4i> &stoplines)
    {
        if(stoplines.size() >= 2)
            state_.stop_flag_ = true;
        else
            state_.stop_flag_ = false;
    }

    void LaneDetector::calculate_slope_and_intercept(const std::vector<cv::Vec4i>& lines, bool is_left)
    {
        double length_sum = 0.0;
        double slope_sum = 0.0;
        double intercept_sum = 0.0;

        for(const cv::Vec4i& line : lines)
        {
            int32_t x1 = line[0];
            int32_t y1 = line[1];
            int32_t x2 = line[2];
            int32_t y2 = line[3];

            if(x2 - x1 == 0)
                continue;

            int32_t diff_y = y2 - y1;
            int32_t diff_x = x2 - x1;
            double slope = static_cast<double>(diff_y) / (diff_x);
            double intercept = y1 + roi_frame_y - slope * x1;
            double line_length = sqrt(diff_y * diff_y) + (diff_x * diff_x);

            length_sum += line_length;
            slope_sum += slope * line_length;
            intercept_sum += intercept * line_length;
        }

        if(std::round(length_sum) != 0)
        {
            if(is_left){
                state_.left_slope_ = slope_sum / length_sum;
                state_.left_intercept_ = intercept_sum / length_sum;
            }
            else{
                state_.right_slope_ = slope_sum / length_sum;
                state_.right_intercept_ = intercept_sum / length_sum;
            }
        }
        else //thers is no lines
        {
            if(is_left){
                state_.left_slope_ = 0;
                state_.left_intercept_ = 0;
            }
            else{
                state_.right_slope_ = 0;
                state_.right_intercept_ = 0;
            }
        }
    }

    void LaneDetector::calculate_pos(bool is_left)
    {
        if(is_left){
            if(std::round(state_.left_slope_) == 0 && std::round(state_.left_intercept_) == 0){
                // state_.left_pos_ = -1; //for refine_pos
                state_.left_pos_ = 0;
            }
            else{
                state_.left_pos_ = static_cast<int32_t>((offset - state_.left_intercept_)/ state_.left_slope_);
                if(state_.left_pos_ < 0 || state_.left_pos_ > frame_width)
                    state_.left_pos_ = 0;
            }
        }
        else{
            if(std::round(state_.right_slope_) == 0 && std::round(state_.right_intercept_) == 0){
                // state_.right_pos_ = frame_width + 1; //for refine_pos
                state_.right_pos_ = frame_width;
            }
            else{
                state_.right_pos_ = static_cast<int32_t>((offset - state_.right_intercept_)/ state_.right_slope_);
                if(state_.right_pos_ > frame_width || state_.right_pos_ < 0)
                    state_.right_pos_ = frame_width;
            }
        }
    }

    void LaneDetector::refine_pos()
    {
        constexpr double k_under_limit = 0.6;
        constexpr double k_upper_limit = 1.0;

        if(state_.left_pos_ < 0){
            if((state_.right_pos_ <= frame_width) && (k_under_limit < abs(state_.right_slope_)) && (abs(state_.right_slope_) < k_upper_limit)){
                state_.left_pos_ = state_.right_pos_ - lane_width;
                state_.left_slope_ = -state_.right_slope_;
                state_.left_intercept_ = offset - state_.left_slope_ * state_.left_pos_;
                if(state_.left_pos_ < 0) state_.left_pos_ = 0;
            }
            else {
                state_.left_pos_ = 0;
            }
        }
        else if(state_.right_pos_ > frame_width){
            if((state_.left_pos_ >= 0) && (k_under_limit < abs(state_.left_slope_)) && (abs(state_.left_slope_) < k_upper_limit)){
                state_.right_pos_ = state_.left_pos_ + lane_width;
                state_.right_slope_ = -state_.left_slope_;
                state_.right_intercept_ = offset - state_.right_slope_ * state_.right_pos_;
                if(state_.right_pos_ > lane_width) state_.right_pos_ = frame_width;
            }
            else {
                state_.right_pos_ = frame_width;
            }
        }
    }
}