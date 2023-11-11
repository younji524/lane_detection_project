# cmake_minimum_required(VERSION 3.0.2)
# project(LaneDetection
#   VERSION 1.0.0
#   LANGUAGES CXX)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_VERBOSE_MAKEFILE ON)

# set(${PROJECT_NAME}_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# set(${PROJECT_NAME}_SOURCE_PATH "${${PROJECT_NAME}_PATH}/src")

# # set(OpenCV_DIR /usr/share/OpenCV) # xycar

# file(GLOB ${PROJECT_NAME}_SOURCE_FILES
#     "${${PROJECT_NAME}_SOURCE_PATH}/*.cpp"
# )

# add_library(LaneDetection
#         ${${PROJECT_NAME}_SOURCE_FILES}
# )

# target_include_directories(${PROJECT_NAME} PUBLIC
#         ${catkin_INCLUDE_DIRS}
#         # ${OpenCV_INCLUDE_DIRS}
#         include
# )

# target_link_libraries(${PROJECT_NAME} PRIVATE
#         ${catkin_LIBRARIES}
#         ${OpenCV_LIBS} # rosbag
#         # ${OpenCV_LIBRARIES} # xycar
# )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(HEADER_FILES
        include/Common.hpp
        include/draw.hpp
        include/ImageProcessor.hpp
        include/KalmanFilter.hpp
        include/LaneDetector.hpp
        include/LaneManager.hpp
        include/PIDController.hpp
        include/XycarController.hpp
)

set(SOURCE_FILES
        src/ImageProcessor.cpp
        src/KalmanFilter.cpp
        src/LaneDetector.cpp
        src/LaneManager.cpp
        src/PIDController.cpp
        src/XycarController.cpp
)

add_library(${PROJECT_NAME}
        ${HEADER_FILES}
        ${SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

