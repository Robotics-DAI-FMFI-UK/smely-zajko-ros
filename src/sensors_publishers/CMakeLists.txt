cmake_minimum_required(VERSION 2.8.3)
project(sensors_publishers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        cv_bridge
        message_types
        std_msgs
        sensor_msgs
        image_transport)

find_package(OpenCV)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES message_types
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/local/rplidar/sdk/sdk/include)

## Declare a C++ library
# add_library(sensors_publishers
#   src/${PROJECT_NAME}/sensors_publishers.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sensors_publishers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(hokuyo_publisher_node
        src/hokuyo_publisher_node.cpp src/hokuyo/AbstractHokuyo.h
        src/hokuyo/Hokuyo.cpp src/hokuyo/Hokuyo.h
        src/hokuyo/HokuyoSynthetic.cpp src/hokuyo/HokuyoSynthetic.h src/hokuyo/HokuyoFiles.cpp src/hokuyo/HokuyoFiles.h)

#add_executable(rplidar_publisher_node src/rplidar_publisher_node.cpp src/rplidar/RpLidar.cpp)
#add_dependencies(rplidar_publisher_node message_types_gencpp)

#add_executable(rplidar_test_node src/rplidar_test_node.cpp)
#add_dependencies(rplidar_test_node message_types_gencpp)

add_executable(hokuyo_subscriber_node src/hokuyo_subscriber_node.cpp)

add_executable(gps_publisher_node src/gps_publisher_node.cpp src/gps/AbstractGps.h src/gps/Gps.cpp src/gps/Gps.h src/gps/GpsFiles.cpp src/gps/GpsFiles.h src/gps/GpsPoints.cpp src/gps/GpsPoints.h)
add_executable(gps_mock_publisher_node src/gps_mock_publisher_node.cpp src/gps/AbstractGps.h src/gps/Gps.cpp src/gps/Gps.h src/gps/GpsFiles.cpp src/gps/GpsFiles.h src/gps/GpsPoints.cpp src/gps/GpsPoints.h)
add_executable(gps_subscriber_node src/gps_subscriber_node.cpp)

add_executable(imu_publisher_node src/imu_publisher_node.cpp src/imu/AbstractImu.h src/imu/Imu.cpp src/imu/Imu.h)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(sbot_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(hokuyo_publisher_node ${catkin_LIBRARIES})
#target_link_libraries(rplidar_publisher_node ${catkin_LIBRARIES} /usr/local/rplidar/sdk/output/Linux/Release/librplidar_sdk.a)
#target_link_libraries(rplidar_test_node ${catkin_LIBRARIES})
target_link_libraries(hokuyo_subscriber_node ${catkin_LIBRARIES})
target_link_libraries(gps_publisher_node ${catkin_LIBRARIES})
target_link_libraries(gps_mock_publisher_node ${catkin_LIBRARIES})
target_link_libraries(gps_subscriber_node ${catkin_LIBRARIES})
target_link_libraries(imu_publisher_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sensors_publishers sbot_publisher_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sbot_publisher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
