cmake_minimum_required(VERSION 2.8)

project(camtest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g")


find_package(OpenCV REQUIRED)
 # find opencv3 OpenCVConfig.cmake
# set(OpenCV_DIR "/usr/local//share/OpenCV")
# find_package(OpenCV 3.4.0 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})


include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(CEDRIVER_CONFIG src/cedriver_config.cpp)

add_library(CEDRIVER_CAM src/cedriver_usb.cpp src/cedriver_cam.cpp)

add_library(CELIB_MadgwickAHRS src/celib_MadgwickAHRS.cpp)

add_library(CELIB_IMG_PROCESS src/celib_img_process.cpp)

add_library(CEDRIVER_IMU  src/cedriver_imu.cpp )
target_link_libraries(CEDRIVER_IMU CELIB_MadgwickAHRS )




## APP
add_executable(CEAPP_RAW_DATA src/ceapp_raw_data.cpp)
target_link_libraries(  CEAPP_RAW_DATA 
                        CEDRIVER_CAM 
                        CEDRIVER_CONFIG 
                        CEDRIVER_IMU   
                        usb-1.0 
                        ${OpenCV_LIBS})

add_executable(CEAPP_STEREO_MATCH src/ceapp_stereo_match.cpp)
target_link_libraries(  CEAPP_STEREO_MATCH 
                        CEDRIVER_CAM 
                        CEDRIVER_CONFIG 
                        CEDRIVER_IMU  
                        CELIB_IMG_PROCESS 
                        usb-1.0 
                        ${OpenCV_LIBS})

add_executable(CEAPP_DISP_FILTER src/ceapp_disp_filter.cpp)
target_link_libraries(  CEAPP_DISP_FILTER 
                        CEDRIVER_CAM 
                        CEDRIVER_CONFIG 
                        CEDRIVER_IMU 
                        CELIB_MadgwickAHRS 
                        CELIB_IMG_PROCESS 
                        usb-1.0 
                        ${OpenCV_LIBS})

## TOOLS
add_executable(CETOOL_CALI_IMU src/cetool_cali_imu.cpp)
target_link_libraries(  CETOOL_CALI_IMU  
                        CEDRIVER_IMU  
                        CEDRIVER_CONFIG 
                        ${OpenCV_LIBS})

add_executable(CETOOL_STEREO_CAPTURE_IMG src/cetool_cali_stereo_capture_img.cpp)
target_link_libraries(  CETOOL_STEREO_CAPTURE_IMG 
                        CEDRIVER_CAM 
                        CEDRIVER_CONFIG 
                        CEDRIVER_IMU 
                        CELIB_IMG_PROCESS 
                        usb-1.0 
                        ${OpenCV_LIBS})

add_executable(CETOOL_CALI_STEREO_CAL src/cetool_cali_stereo_cal.cpp)
target_link_libraries(  CETOOL_CALI_STEREO_CAL 
                        CEDRIVER_CAM 
                        CEDRIVER_CONFIG 
                        CEDRIVER_IMU 
                        usb-1.0 
                        ${OpenCV_LIBS})
                        
            
                        
