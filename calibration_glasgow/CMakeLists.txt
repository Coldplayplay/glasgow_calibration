cmake_minimum_required(VERSION 2.8.3)
project(calibration_glasgow)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_msgs
  cv_bridge
  image_transport
  smach
  smach_ros  
  tf
  message_generation
 
)

find_package(PCL 1.2 REQUIRED)

catkin_python_setup()

add_service_files(
    FILES
    TargetProcess.srv
    HandEyeCalibration.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    DEPENDS cv_bridge image_transport roscpp rospy smach smach_ros std_msgs tf message_runtime sensor_msgs geometry_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(OpenCV_DIR /usr/local/opencv320/share/OpenCV)
find_package(OpenCV REQUIRED)
#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include)
#link_directories(${OpenCV_LIBRARY_DIRS} /usr/local/opencv320/lib)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake")

find_package(cvsba REQUIRED)



set(CALIBRATION_SOURCES
    src/calibration_services.cpp
    src/camera_calibration.cpp
    src/handeye_calibration.cpp
    #src/robot_helper.cpp
)

add_executable(calibration_services ${CALIBRATION_SOURCES})
target_link_libraries(calibration_services boost_signals)
target_link_libraries(calibration_services ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(calibration_services ${PROJECT_NAME}_gencpp)

add_executable(pub_robot_trans          src/robot_helper.cpp)
target_link_libraries(pub_robot_trans   boost_signals)
target_link_libraries(pub_robot_trans   ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pub_robot_trans        ${PROJECT_NAME}_gencpp)

add_executable(call_handeye_calib          src/handeye_calib_call.cpp)
target_link_libraries(call_handeye_calib   boost_signals)
target_link_libraries(call_handeye_calib   ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(call_handeye_calib        ${PROJECT_NAME}_gencpp)

add_executable(getpose          src/getPose.cpp)
target_link_libraries(getpose   boost_signals)
target_link_libraries(getpose   ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(getpose        ${PROJECT_NAME}_gencpp)

add_executable(record          src/record_joint.cpp)
target_link_libraries(record   boost_signals)
target_link_libraries(record   ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(record        ${PROJECT_NAME}_gencpp)


