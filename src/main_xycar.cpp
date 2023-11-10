// system header
#include <cstdint>

// user defined header
#include "LaneDetection/Common.hpp"
#include "LaneDetection/LaneManager.hpp"

int32_t main(int32_t argc, char** argv)
{
    XyCar::PREC p_gain = 1.0;
    XyCar::PREC i_gain = 1.0;
    XyCar::PREC d_gain = 1.0;

    ros::init(argc, argv, "main_xycar");
    XyCar::LaneManager laneManager(p_gain, i_gain, d_gain);

    laneManager.run();

    return 0;
}
