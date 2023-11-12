#include "LaneManager.hpp"
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

            State lane_state = detector_.find_state(canny_image);

            // std::cout << "lpos: " << lane_state.left_pos_ << std::endl;
            // std::cout << "rpos: " << lane_state.right_pos_ << std::endl;
            // std::cout << "stop: " << lane_state.stop_flag_ << std::endl;

            int32_t error = k_frame_width / 2 - static_cast<int32_t>((lane_state.right_pos_ + lane_state.left_pos_) / 2);

            PREC angle = pid_controller_.compute_angle(error);
            // std::cout <<"angle: " << angle << std::endl;

            xycar_controller.control(angle);

            cv::imshow("image_", image_);
            cv::imshow("canny_image", canny_image);

            cv::waitKey(1);
        }
    }

}
