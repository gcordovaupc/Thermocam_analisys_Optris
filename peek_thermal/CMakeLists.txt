cmake_minimum_required(VERSION 2.8.3)
project(peek_thermal)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    sensor_msgs
    cv_bridge
    dynamic_reconfigure
)

find_package( OpenCV 3 REQUIRED )

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

#dynamic reconfigure
generate_dynamic_reconfigure_options(
  config/hsv_parameters.cfg
)

#service example
#add_service_files(
#  FILES
#  GripPosition.srv
#)

#example for message generation
#generate_messages(DEPENDENCIES insert_msg_pkg)

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
  INCLUDE_DIRS
    include
#    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
#  DEPENDS
#    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
   src/CameraInfoParser.cpp
   src/ConvertToCVimage.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/PeekThermal.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
### Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_crop_plant_standard.cpp
#  test/CameraInfoParserTest.cpp)
#endif()

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif()
