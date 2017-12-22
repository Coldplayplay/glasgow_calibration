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

find_package(OpenCV)
#set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV/")
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

add_executable(robot_trans_pub          src/robot_helper.cpp)
target_link_libraries(robot_trans_pub   boost_signals)
target_link_libraries(robot_trans_pub   ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(robot_trans_pub        ${PROJECT_NAME}_gencpp)





