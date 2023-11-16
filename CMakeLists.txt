cmake_minimum_required(VERSION 3.0.2)
project(lane_detection
  VERSION 1.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  xycar_msgs
)

find_package(yaml-cpp REQUIRED)

# find_package(OpenCV 4.5.5 REQUIRED HINTS ~/OpenCV4/install/lib/cmake/opencv4)
find_package(OpenCV 4.5.5 REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/OpenCV/install/lib/cmake/opencv4)

catkin_package()

add_subdirectory(modules)

add_executable(main_xycar src/main_xycar.cpp)

target_link_libraries(main_xycar PRIVATE
  ${PROJECT_NAME})

add_executable(teleop_xycar test/teleop_xycar.cpp)
target_link_libraries(teleop_xycar PRIVATE
  ${PROJECT_NAME}
)

if(CMAKE_BUILD_TYPE MATCHES Debug)
   add_definitions(-DDEBUG=1)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)