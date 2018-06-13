cmake_minimum_required(VERSION 2.8.3)
project(cooleye_d1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  message_filters
)

find_package(OpenCV REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS  sensor_msgs  std_msgs  cv_bridge  image_transport  message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
 ${catkin_INCLUDE_DIRS}
)

add_library(CEDRIVER_CONFIG src/cedriver_config.cpp)

add_library(CEDRIVER_CAM src/cedriver_usb.cpp src/cedriver_cam.cpp)

add_library(CELIB_MadgwickAHRS src/celib_MadgwickAHRS.cpp)

add_library(CEDRIVER_IMU  src/cedriver_imu.cpp )
target_link_libraries(CEDRIVER_IMU CELIB_MadgwickAHRS )

add_library(CEROS_CAM_D1_DRIVER src/ceros_cam_d1_driver.cpp) 
target_link_libraries(CEROS_CAM_D1_DRIVER CEDRIVER_CONFIG CEDRIVER_CAM CEDRIVER_IMU CELIB_MadgwickAHRS usb-1.0 ${OpenCV_LIBS} ${catkin_LIBRARIES})


add_executable(CEROS_CAM_D1_NODE src/ceros_cam_d1_node.cpp) 
target_link_libraries(CEROS_CAM_D1_NODE CEROS_CAM_D1_DRIVER)



