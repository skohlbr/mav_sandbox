cmake_minimum_required(VERSION 2.8.3)
project(mav_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  cv_debug_provider
  cv_image_proc
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

generate_dynamic_reconfigure_options(cfg/MavTracker.cfg)

find_package(OpenCV REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dial_gauge_proc
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport pcl_ros roscpp sensor_msgs tf
#  DEPENDS system_lib
)

add_library(mav_tracker src/mav_tracker.cpp)
target_link_libraries(mav_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(mav_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mav_tracker_node src/mav_tracker_node.cpp)
target_link_libraries(mav_tracker_node mav_tracker)
#add_dependencies(mav_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(mav_tracker_nodelet src/mav_tracker_nodelet.cpp)
target_link_libraries(mav_tracker_nodelet mav_tracker)