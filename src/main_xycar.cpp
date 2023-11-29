/**
 * @file main.cpp
 * @author Nahye Kim (nahelove03@gmail.com) Dongwook Heo (hdwook3918@gmail.com)
 * @brief Main entry point for the XyCar lane detection and control program.
 * @version 1.0.0
 * @date 2023-11-09
 * @copyright Copyright (c) 2023 I_On_Car, All Rights Reserved.
 */
// System header
#include <cstdint>
// User defined header
#include "Common.hpp"
#include "LaneManager.hpp"

int32_t main(int32_t argc, char **argv)
{
  ros::init(argc, argv, "main_xycar");
  XyCar::LaneManager laneManager;
  laneManager.run();

  return 0;
}
