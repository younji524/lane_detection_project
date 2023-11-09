cmake_minimum_required(VERSION 3.0.2)
project(LaneDetection
  VERSION 1.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(${PROJECT_NAME}_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(${PROJECT_NAME}_SOURCE_PATH "${${PROJECT_NAME}_PATH}/src")

set(OpenCV_DIR /usr/share/OpenCV)

file(GLOB ${PROJECT_NAME}_SOURCE_FILES
    "${${PROJECT_NAME}_SOURCE_PATH}/*.cpp"
)

add_library(LaneDetection
        ${${PROJECT_NAME}_SOURCE_FILES}
)

find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

if(OpenCV_FOUND)
    message(STATUS "OpenCV Found! - ${OpenCV_DIR}")
endif()

message("Source files: ${${PROJECT_NAME}_SOURCE_FILES}")

# target_include_directories(${PROJECT_NAME} PUBLIC
#         include
#         ${catkin_INCLUDE_DIRS}
#         ${OpenCV_INCLUDE_DIRS}
# )

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
