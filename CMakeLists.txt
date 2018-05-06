CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 14)
PROJECT(place_detection_isl)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Sql)
FIND_PACKAGE(OpenCV REQUIRED)

CATKIN_PACKAGE()

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/bubble)
MESSAGE( STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})

SET(bubble_space_SOURCES
  src/bubble/bubbleprocess.cpp
  src/database/databasemanager.cpp
  src/imageprocess/imageprocess.cpp
  src/Utility/BasePoint.cpp
  src/Utility/Place.cpp
  src/Utility/TemporalWindow.cpp
)

SET(bubble_space_HEADERS
  src/bubble/bubbleprocess.h
  src/database/databasemanager.h
  src/imageprocess/imageprocess.h
  src/Utility/BasePoint.h
  src/Utility/Place.h
  src/Utility/TemporalWindow.h
  src/Utility/PlaceDetector.h
)

ADD_LIBRARY(${PROJECT_NAME} src/main.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} opencv_core opencv_imgcodecs opencv_imgproc opencv_highgui ${catkin_LIBRARIES})
ADD_EXECUTABLE(place_detection_node src/main.cpp ${bubble_space_SOURCES} ${bubble_space_HEADERS})
TARGET_LINK_LIBRARIES(place_detection_node ${PROJECT_NAME} Qt5::Core Qt5::Sql)
