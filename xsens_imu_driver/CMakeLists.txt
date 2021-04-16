cmake_minimum_required(VERSION 2.8.3)
project(xsens_imu_driver)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  serial
)


add_message_files(
  DIRECTORY msg
  FILES
  synin.msg
  imu_pressure.msg
  imu_magneticField.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/xsens/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for the xsens library
link_directories(
  /usr/local/xsens/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(xsens_imu_driver src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(xsens_imu_driver
  ${catkin_LIBRARIES}
  xsensdeviceapi
  xstypes
)

add_dependencies(xsens_imu_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

