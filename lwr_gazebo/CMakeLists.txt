cmake_minimum_required(VERSION 2.8.3)
project(lwr_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

 add_executable(lwr_gazebo src/human_controller.cpp)

 add_dependencies(lwr_gazebo human_controller_cpp)

 target_link_libraries(lwr_gazebo ${catkin_LIBRARIES})
