#include "XycarController.hpp"

namespace XyCar
{
    XycarController::XycarController()
    {
        // publisher_ = node_handler_.advertise<xycar_msgs::xycar_motor>("xycar_motor", 1);
    }

    xycar_msgs::xycar_motor XycarController::control(XyCar::PREC angle)
    {
        // publisher_.publish(make_motor_message(angle, decide_speed(angle)));
        return make_motor_message(angle, decide_speed(angle));
    }

    PREC XycarController::decide_speed(PREC angle)
    {
        // when xycar turn
        if(std::abs(angle) > 10)
        {
            speed_ -= k_step_speed_;
            speed_ = std::max(speed_, k_min_speed_);
        }
        // when xycar go straight
        else
        {
            speed_ += k_step_speed_;
            speed_ = std::min(speed_, k_max_speed_);
        }

        return speed_;
    }

    xycar_msgs::xycar_motor XycarController::make_motor_message(PREC angle, PREC speed)
    {
        xycar_msgs::xycar_motor motor_message;
        motor_message.header.stamp = ros::Time::now();
        motor_message.angle = std::round(angle);
        motor_message.speed = std::round(speed);

        return motor_message;
    }
}
