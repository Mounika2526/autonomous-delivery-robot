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

#############
## Install ##
#############

## Install Python nodes
catkin_install_python(PROGRAMS
  scripts/fake_publishers.py
  scripts/pose_broadcaster.py
  scripts/fake_laser.py
  scripts/nav_sim.py
  src/access_database.py
  src/gps.py
  src/sim_nav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
