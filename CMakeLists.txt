cmake_minimum_required(VERSION 3.0.2)
project(autonomous-delivery-robot)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

###################################
## Catkin Package Configuration  ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ executables (your ROS nodes)
add_executable(main_adr src/main.cpp)
target_link_libraries(main_adr
  ${catkin_LIBRARIES}
)

add_executable(delivery_adr src/delivery.cpp)
target_link_libraries(delivery_adr
  ${catkin_LIBRARIES}
)

add_executable(obs_adr src/obs_main.cpp)
target_link_libraries(obs_adr
  ${catkin_LIBRARIES}
)

add_executable(main_cmdvel src/main_cmdvel.cpp)
target_link_libraries(main_cmdvel ${catkin_LIBRARIES})

add_executable(obs_cmdvel src/obs_cmdvel.cpp)
target_link_libraries(obs_cmdvel ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install Python nodes
catkin_install_python(PROGRAMS
  scripts/pose_broadcaster.py
  src/access_database.py
  src/gps.py
  src/sim_nav.py
  scripts/odom_to_pose2d.py
  scripts/nav_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
