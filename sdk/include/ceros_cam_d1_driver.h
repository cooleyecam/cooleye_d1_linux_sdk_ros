#ifndef CEROS_CAM_D1_DRIVER_H
#define CEROS_CAM_D1_DRIVER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<boost/thread/thread.hpp>
#include <sys/time.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class CoolEyeCamD1
{
public:
    CoolEyeCamD1(ros::NodeHandle nh,char path[]);
    ~CoolEyeCamD1();

    bool start_cam_D1_capture();
    bool start_cam_D1_preprocess();
    bool start_imu_icm26089();

    void img_data_stream();
    void imu_data_stream();
    void exectu();


    boost::thread *cam_thread_;
    boost::thread *imu_thread_;

    cv::Mat img_left_, img_right_;
    cv_bridge::CvImage left_bridge_, right_bridge_;

    image_transport::Publisher pub_caml_, pub_camr_;
    ros::Publisher pub_imu_;

    bool cam_thread_stop_run_;
    bool imu_thread_stop_run_;
};

#endif
