cmake_minimum_required(VERSION 3.0.2)
project(mapper_sub)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  move_base_msgs
  json_transport
)
#find_package(nlohmann_json 3.10.4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapper_sub
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  /usr/local/include
  /paho.mqtt.cpp/lib
  /usr/local/lib
  /paho.mqtt.cpp
  /paho.mqtt.cpp/src
  /paho.mqtt.cpp/src/mqtt
  /paho.mqtt.cpp/lib
  /catkin_workspace/src/mapper_sub/include/nlohmann
  include #${catkin_INCLUDE_DIRS}
  #include
  ${catkin_INCLUDE_DIRS}
  include
)
link_directories(
	${catkin_LIB_DIRS}
	lib
	)

add_executable(sub src/5.cpp)
add_dependencies(sub ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(sub 
	${catkin_LIBRARIES}
	pthread
	libpaho-mqttpp3.so.1
	libpaho-mqtt3as.so.1
	libpaho-mqtt3c.so.1
	libpaho-mqtt3a.so.1
)
