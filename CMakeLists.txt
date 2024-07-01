cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_player)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Set the search path for AUTOUIC
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${Qt5Core_DEFINITIONS} ${Qt5Gui_DEFINITIONS} ${Qt5Widgets_DEFINITIONS} ${PCL_DEFINITIONS})

set(UI_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/ui/widget.ui
)

qt5_wrap_ui(UIC_HEADERS ${UI_FILES}) 

add_executable(${PROJECT_NAME}_node
  src/main.cc
  src/widget.cc
  src/pointcloudpublisher.cc
  src/dataset/kitti_dataset.cc
  src/dataset/nclt_dataset.cc
  ${UIC_HEADERS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  ${PCL_LIBRARIES}
)