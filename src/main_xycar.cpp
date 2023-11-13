// system header
#include <cstdint>

// user defined header
#include "Common.hpp"
#include "LaneManager.hpp"

int32_t main(int32_t argc, char** argv)
{
    XyCar::PREC p_gain = 0.6;
    XyCar::PREC i_gain = 0.0006;
    XyCar::PREC d_gain = 0.025;

    ros::init(argc, argv, "main_xycar");
    XyCar::LaneManager laneManager(p_gain, i_gain, d_gain);

    laneManager.run();

    return 0;
}
