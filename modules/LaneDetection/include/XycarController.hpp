#ifndef LANE_DETECTION__XYCARCONTROLLER_HPP
#define LANE_DETECTION__XYCARCONTROLLER_HPP

// system header
#include <algorithm>
#include <cmath>

// third party header
#include "ros/ros.h"
#include "xycar_msgs/xycar_motor.h"

// user defined header
#include "Common.hpp"

namespace XyCar
{
class XycarController
{
public:
    XycarController();

    /**
     * @details Transmit the speed & angle to xycar motor driver.
     * @param[in] angle Steering angle of xycar.\n (-20~20[deg] are mapped to -50~50 values)
     * @return void
     */
    xycar_msgs::xycar_motor control(PREC angle);

private:
    /**
     * @details Decide the speed of xycar based on the 'angle' value.
     * @param[in] angle Steering angle of xycar.\n (-20~20[deg] are mapped to -50~50 values)
     * @return XyCar::PREC
     */
    PREC decide_speed(PREC angle);

    /**
     * @details Create a xycar_motor message used to drive xycar.
     * @param angle Steering angle of xycar.\n (-20~20[deg] are mapped to -50~50 values)
     * @param speed Speed of xycar.
     * @return xycar_msgs::xycar_motor
     */
    xycar_msgs::xycar_motor make_motor_message(PREC angle, PREC speed);


    // ros::NodeHandle node_handler_;
    // ros::Publisher publisher_;
    static constexpr PREC k_max_speed_ = 10;
    static constexpr PREC k_min_speed_ = 5;
    static constexpr PREC k_step_speed_ = 1;
    PREC speed_ = k_min_speed_;
};
} // XyCar

#endif // LANE_DETECTION__XYCARCONTROLLER_HPP
