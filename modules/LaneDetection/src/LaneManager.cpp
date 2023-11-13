#include "LaneManager.hpp"
#include <iostream>
#include "draw.hpp"

namespace XyCar
{
    LaneManager::LaneManager(PREC p_gain, PREC i_gain, PREC d_gain) : pid_controller_(p_gain, i_gain, d_gain)
    {
        publisher_ = node_handler_.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 1);
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

            cv::Mat draw_image = image_.clone();

            cv::Mat canny_image = image_processor_.process(image_);
            State lane_state = detector_.find_state(canny_image);

            draw_line_slope(draw_image, lane_state.left_slope_, lane_state.left_intercept_, cv::Scalar(255,0,0));
            draw_line_slope(draw_image, lane_state.right_slope_, lane_state.right_intercept_, cv::Scalar(255,0,0));

            // std::cout << "lpos: " << lane_state.left_pos_ << std::endl;
            // std::cout << "rpos: " << lane_state.right_pos_ << std::endl;
            // std::cout << "stop: " << lane_state.stop_flag_ << std::endl;

            int32_t centor = static_cast<int32_t>((lane_state.right_pos_ + lane_state.left_pos_) / 2);
            int32_t error = centor - (k_frame_width >> 1);


            // cv::rectangle(draw_image, cv::Rect(cv::Point(5, 5), cv::Point(105, 105)), cv::Scalar(255,0,0), 2);

            cv::rectangle(draw_image, cv::Rect(cv::Point(centor - 5, k_offset - 5), cv::Point(centor + 15, k_offset - 15)), cv::Scalar(255,0,0), 2);
            // cv::rectangle(draw_image, cv::Rect(cv::Point(k_frame_width / 2 - 5, k_offset - 5), cv::Point(k_frame_width / 2 + 5, k_offset - 5)), cv::Scalar(0,0,255), 2);

            PREC angle = pid_controller_.compute_angle(error);
            std::cout <<"error: " << error << std::endl;
            std::cout <<"angle: " << angle << std::endl;

            publisher_.publish(xycar_controller.control(angle));

            cv::imshow("draw_image", draw_image);
            // cv::imshow("canny_image", canny_image);

            cv::waitKey(1);
        }
    }

}
