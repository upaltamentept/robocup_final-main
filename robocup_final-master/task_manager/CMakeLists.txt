cmake_minimum_required(VERSION 2.8.3)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

add_service_files(
   FILES
   Grasp.srv
   PoseObject.srv
   PosePlace.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()




install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/task_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
endif()
