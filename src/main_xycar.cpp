// system header
#include <cstdint>
// user defined header
#include "Common.hpp"
#include "LaneManager.hpp"

int32_t main(int32_t argc, char **argv)
{
  ros::init(argc, argv, "main_xycar");
  XyCar::LaneManager laneManager;
  laneManager.run();

  return 0;
}
