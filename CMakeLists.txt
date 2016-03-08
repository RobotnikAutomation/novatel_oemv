cmake_minimum_required(VERSION 2.8.3)
project(novatel_oemv)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    roslib
    tf
    nav_msgs 
    std_msgs
    std_srvs
    geometry_msgs 
    sensor_msgs 
    message_generation
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   bestxyza.msg
   bestposa.msg
   ionutca.msg
   pdpxyza.msg
   pdpposa.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES summit_xl_robot_control
  CATKIN_DEPENDS 
    roscpp 
    roslib
    tf
    nav_msgs 
    std_msgs
    geometry_msgs 
    sensor_msgs
    message_runtime 
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include  
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp executable
add_executable(gps_node src/gps_node.cc src/novatel_oemv1gl1.cc src/SerialDevice.cc src/Component.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(gps_node rt ${catkin_LIBRARIES} )
