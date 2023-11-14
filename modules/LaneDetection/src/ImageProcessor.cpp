#include "ImageProcessor.hpp"

namespace XyCar
{
    void ImageProcessor::set_configuration(const YAML::Node& config)
    {
        roi_frame_y = config["IMAGE"]["ROI_Y_POS"].as<uint32_t>();
        frame_width = config["IMAGE"]["WIDTH"].as<uint32_t>();
        roi_frame_height = config["IMAGE"]["ROI_HEIGHT"].as<uint32_t>();
    }

    cv::Mat ImageProcessor::process(const cv::Mat &frame)
    {
        frame.copyTo(cropped_frame_);
        cropped_frame_ = cropped_frame_(cv::Rect(0, roi_frame_y, frame_width, roi_frame_height));
        // cv::imshow("crop", cropped_frame_);

        // gray image
        cv::cvtColor(cropped_frame_, cropped_frame_, cv::COLOR_BGR2GRAY);

        // binarization
        // cv::equalizeHist(cropped_frame_, cropped_frame_);
        // cv::threshold(cropped_frame_, cropped_frame_, 65, 255, cv::THRESH_BINARY_INV);
        // cv::imshow("crop", cropped_frame_);

        // lidar mask
        // cv::bitwise_and(cropped_frame_, mask_lidar_(cv::Rect(0,(mask_lidar_.rows>>3)*5,mask_lidar_.cols,(mask_lidar_.rows>>3)*3)), cropped_frame_);

        // blur (gaussian)
        cv::GaussianBlur(cropped_frame_, cropped_frame_, cv::Size(), 2);

        // canny edge
        cv::Canny(cropped_frame_, cropped_frame_, 50, 150);

        return cropped_frame_;
    }

} // XyCar
