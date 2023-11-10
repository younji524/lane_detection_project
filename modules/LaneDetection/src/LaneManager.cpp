#include "LaneDetection/LaneManager.hpp"
#include <iostream>
namespace XyCar
{
    LaneManager::LaneManager(PREC p_gain, PREC i_gain, PREC d_gain) : pid_controller_(p_gain, i_gain, d_gain)
    {
        subscriber_ = node_handler_.subscribe("/usb_cam/image_raw/", 1, &LaneManager::image_callback, this);
    }

    void LaneManager::image_callback(const sensor_msgs::Image& message)
    {
        cv::Mat camera_image = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
        cv::cvtColor(camera_image, image_, cv::COLOR_RGB2BGR);
    }

    void LaneManager::run()
    {
        ros::Rate rate(33);
        while (ros::ok())
        {
            ros::spinOnce();
            std::cout << image_.size() << std::endl;

            if(image_.empty())
                continue;

            cv::Mat canny_image = image_processor_.process(image_);

            std::tuple<int32_t, int32_t, bool> output_detector;
            output_detector = detector_.findPos(canny_image);
            int32_t left_pos = std::get<0>(output_detector);
            int32_t right_pos = std::get<1>(output_detector);
            bool is_stop = std::get<2>(output_detector);

            // std::cout << "lpos: " << left_pos << std::endl;
            // std::cout << "rpos: " << right_pos << std::endl;
            // std::cout << "stop: " << is_stop << std::endl;

            int32_t error = k_frame_width / 2 - static_cast<int32_t>((right_pos + left_pos) / 2);

            PREC angle = pid_controller_.computeAngle(error);
            // std::cout <<"angle: " << angle << std::endl;

            xycar_controller.control(angle);

            cv::imshow("image_", image_);
            cv::imshow("canny_image", canny_image);

            cv::waitKey(1);
        }
    }

}
