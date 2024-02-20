cmake_minimum_required(VERSION 3.0.2)
project(obstacle_converter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  project_msgs
  roscpp
  std_msgs
  moveit_msgs
  moveit_ros_planning_interface
  geometric_shapes
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_converter
  CATKIN_DEPENDS 
    geometry_msgs
    project_msgs 
    roscpp 
    std_msgs
    interactive_markers
    moveit_ros_planning_interface
  DEPENDS 
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_node src/application/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
