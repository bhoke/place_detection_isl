cmake_minimum_required(VERSION 2.8.3)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

project(place_detection_isl)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

find_package(Qt4 REQUIRED)
set(QT_USE_QTSQL TRUE)

find_package( OpenCV REQUIRED )

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/bubble)
message( STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})


set(bubble_space_SOURCES src/bubble/bubbleprocess.cpp src/database/databasemanager.cpp)
set(bubble_space_NSOURCES src/imageprocess/imageprocess.cpp src/Utility.cpp)
set(bubble_space_HEADERS src/bubble/bubbleprocess.h src/database/databasemanager.h)
set(bubble_space_NHEADERS src/imageprocess/imageprocess.h src/Utility.h)


QT4_WRAP_CPP(bubble_space_HEADERS_MOC ${bubble_space_HEADERS})
qt4_automoc(${bubble_space_SOURCES})

#ADD_EXECUTABLE(bubble_space ${bubble_space_SOURCES} ${bubble_space_HEADERS_MOC})
#TARGET_LINK_LIBRARIES(bubble_space ${QT_LIBRARIES})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} opencv_highgui opencv_imgproc opencv_core ${catkin_LIBRARIES})

add_executable(place_detection_node src/main.cpp ${bubble_space_SOURCES} ${bubble_space_NSOURCES} ${bubble_space_NHEADERS} ${bubble_space_HEADERS_MOC})
target_link_libraries(place_detection_node ${PROJECT_NAME} ${QT_LIBRARIES})
