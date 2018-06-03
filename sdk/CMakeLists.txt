cmake_minimum_required(VERSION 2.8)

project(camtest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g")

find_package(OpenCV REQUIRED)


include_directories("${PROJECT_SOURCE_DIR}/include")


add_library(CEDRIVER_CONFIG src/cedriver_config.cpp)

add_library(CEDRIVER_CAM src/cedriver_usb.cpp src/cedriver_cam.cpp)

add_library(CELIB_MadgwickAHRS src/celib_MadgwickAHRS.cpp)


add_library(CEDRIVER_IMU  src/cedriver_imu.cpp )
target_link_libraries(CEDRIVER_IMU CELIB_MadgwickAHRS )

add_executable(CEAPP_MAIN src/ceapp_main.cpp)
target_link_libraries(CEAPP_MAIN CEDRIVER_CAM CEDRIVER_CONFIG CEDRIVER_IMU CELIB_MadgwickAHRS  usb-1.0 ${OpenCV_LIBS})

add_executable(CEAPP_CALI_IMU src/ceapp_cali_imu.cpp)
target_link_libraries(CEAPP_CALI_IMU  CEDRIVER_IMU  CEDRIVER_CONFIG ${OpenCV_LIBS})
