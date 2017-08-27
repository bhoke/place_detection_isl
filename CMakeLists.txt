cmake_minimum_required(VERSION 2.8.3)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_CXX_STANDARD 14)
project(place_detection_isl)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
find_package(Qt5Sql)

find_package( OpenCV REQUIRED )

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/bubble)
message( STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})

set(bubble_space_SOURCES src/bubble/bubbleprocess.cpp src/database/databasemanager.cpp)# src/pcprocessing.cpp)
set(bubble_space_NSOURCES src/bubble/dfc.cpp src/bubble/invariants.cpp src/imageprocess/imageprocess.cpp src/Utility.cpp)
set(bubble_space_HEADERS src/bubble/bubbleprocess.h src/database/databasemanager.h)# include/bubble_space/pcprocessing.h)
set(bubble_space_NHEADERS src/imageprocess/imageprocess.h src/Utility.h)# include/bubble_space/pcprocessing.h)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} opencv_highgui opencv_imgproc opencv_core opencv_imgcodecs ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} another_library)
#find_package(Boost REQUIRED COMPONENTS components); ()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(place_detection_node src/main.cpp ${bubble_space_SOURCES} ${bubble_space_NSOURCES} ${bubble_space_NHEADERS} ${bubble_space_HEADERS_MOC})
target_link_libraries(place_detection_node ${PROJECT_NAME} Qt5::Widgets Qt5::Sql)
