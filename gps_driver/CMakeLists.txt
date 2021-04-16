cmake_minimum_required(VERSION 3.0.2)
project(gps_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE=Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
  # nk_nav_msg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  GPCHC.msg
  IMURaw.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES gps_driver
  CATKIN_DEPENDS geometry_msgs nk_nav_msg roscpp rospy sensor_msgs std_msgs tf message_generation
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${LINK_DIR})

add_executable(ground_truth_node src/gpsResolve.cpp)
target_link_libraries(ground_truth_node
  ${catkin_LIBRARIES}
) # 用于解析GPCHC数据--groundtruth

# src/libpcanbasic.c
add_executable(can_rawIMU_node src/canResolve.cpp )
target_link_libraries(can_rawIMU_node
  ${catkin_LIBRARIES}
  /usr/lib/libpcanbasic.so
)

# # 检测系统状态
# add_executable(system_detect_node src/systemDetect.cpp)
# target_link_libraries(system_detect_node
#   ${catkin_LIBRARIES}
# )

# 同步GPS--电脑时间
add_executable(system_syne_node src/gpsSetSyne.cpp)
target_link_libraries(system_syne_node
  ${catkin_LIBRARIES}
)

# 测试ros时间变化
add_executable(test_node src/test.cpp)
target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_executable(gpchc_node src/gpsResolve_fix_hz.cpp)
target_link_libraries(gpchc_node
  ${catkin_LIBRARIES}
) # 用于解析GPCHC数据--groundtruth