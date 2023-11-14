#ifndef LANE_DETECTION__IMAGEPROCESSOR_HPP
#define LANE_DETECTION__IMAGEPROCESSOR_HPP

#include "opencv2/opencv.hpp"
#include <cstdint>
#include "Common.hpp"


namespace XyCar
{
/**
 * @details Class responsible for pre-processing of the image for lane detection.
 */
class ImageProcessor
{
public:
    using Ptr = ImageProcessor*;

    ImageProcessor(const YAML::Node& config) {set_configuration(config);}

    /**
     * @details Perform the tasks below for the initial configuration.\n
     *          1. Bring up the LiDAR mask.\n
     */
    // void init();

    /**
     * @details Perfrom the image preprocessing tasks below.\n
     * 1. Crop the image for ROI settings.\n 2. Convert to grayscale.\n
     * 3. Perform Histogram Equalization & Binarization.\n 4. Apply the LiDAR mask.\n
     * 5. Perform a Gaussian Blur.\n 6. Apply a Canny Algorithm.\n
     * @param[in] frame The original video frame.
     * return cv::Mat
     */
    cv::Mat process(const cv::Mat& frame);

private:
    cv::Mat cropped_frame_;
    // cv::Mat mask_lidar_;
    uint32_t roi_frame_y;
    uint32_t frame_width;
    uint32_t roi_frame_height;

    /**
     * @details set values from config
     * @param[in] config config.yaml file
     * @return void
     */
    void set_configuration(const YAML::Node& config);
};
} // XyCar

#endif // LANE_DETECTION__IMAGEPROCESSOR_HPP
