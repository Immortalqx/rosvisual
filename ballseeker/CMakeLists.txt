cmake_minimum_required(VERSION 3.0.2)
project(ballseeker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/SetParam.cfg
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(OpenCV_DIR   /usr/local/share/OpenCV4)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}  src/seek.cpp)

add_dependencies(
   ${PROJECT_NAME} 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)



target_link_libraries(
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)

add_executable(seekernode src/seek.cpp include/ballseeker/seek.hpp src/seekernode.cpp)  
target_link_libraries(seekernode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)  

# make sure configure headers are built before any node using them
add_dependencies(seekernode ${PROJECT_NAME}_gencfg)
