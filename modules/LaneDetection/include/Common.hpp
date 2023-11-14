#ifndef LANE_DETECTION__COMMON_HPP
#define LANE_DETECTION__COMMON_HPP

#include <cstdint>
#include <yaml-cpp/yaml.h>

namespace XyCar
{
    using PREC = double;

    /**
     * @details  A structure that represents the state of the lanes.
     */
    struct State
    {
        PREC left_slope_;
        PREC left_intercept_;
        PREC right_slope_;
        PREC right_intercept_;
        PREC left_pos_;
        PREC right_pos_;
        bool stop_flag_;
    };
} // XyCar

#endif // LANE_DETECTION__COMMON_HPP
