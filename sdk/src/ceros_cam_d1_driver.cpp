#include "ceros_cam_d1_driver.h"

#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_imu.h"
#include "cedriver_config.h"
#include "threadsafe_queue.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

extern threadsafe_queue<d1_img_output_pkg *> img_pkg_list_d1;
extern threadsafe_queue<icm20689_pkg *> icm20689_pkg_list;

extern threadsafe_queue<s1_img_output_pkg *> img_pkg_list_s1num0;
extern threadsafe_queue<s1_img_output_pkg *> img_pkg_list_s1num1;
extern threadsafe_queue<s1_img_output_pkg *> img_pkg_list_s1num2;


CoolEyeCamD1::CoolEyeCamD1(ros::NodeHandle nh, char path[]):cam_thread_stop_run_(false),imu_thread_stop_run_(false)
{

    ce_config_load_settings(path);

    pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu0_icm20689",ce_config_get_cf_imu_icm20689_sample_rate());

    image_transport::ImageTransport imageTransL(nh);
    pub_caml_ = imageTransL.advertise("/cooleyed1/left/image_raw", 1);

    image_transport::ImageTransport imageTransR(nh);
    pub_camr_ = imageTransR.advertise("/cooleyed1/right/image_raw", 1);


    image_transport::ImageTransport imageTransS1N0(nh);
    pub_cams1n0_ = imageTransS1N0.advertise("/cooleyes1/num0/image_raw", 1);

    image_transport::ImageTransport imageTransS1N1(nh);
    pub_cams1n1_ = imageTransS1N0.advertise("/cooleyes1/num1/image_raw", 1);

    image_transport::ImageTransport imageTransS1N2(nh);
    pub_cams1n2_ = imageTransS1N0.advertise("/cooleyes1/num2/image_raw", 1);




    img_left_.create(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    img_right_.create(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    left_bridge_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left_);
    right_bridge_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right_);




    img_s1n0_.create(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    s1n0_bridge_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_s1n0_);

    img_s1n1_.create(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    s1n1_bridge_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_s1n1_);
    
    img_s1n2_.create(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    s1n2_bridge_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_s1n2_);


}

bool CoolEyeCamD1::start_cam_D1_capture()
{
    return ce_cam_capture_init() < 0 ? false : true;
}


bool CoolEyeCamD1::start_cam_D1_preprocess()
{
    return ce_cam_preprocess_init() < 0 ? false : true;
}



bool CoolEyeCamD1::start_imu_icm26089()
{
    return ce_imu_capture_init() < 0 ? false : true;
}



void CoolEyeCamD1::img_data_stream ( )
{
    ros::NodeHandle cam_n;

    ros::Rate loop_rate(1000);

    sensor_msgs::ImagePtr msg_l, msg_r, msg_s1n0, msg_s1n1, msg_s1n2;
    ros::Time msg_time;
    double left_timestamp, right_timestamp, s1n0_timestamp, s1n1_timestamp, s1n2_timestamp;

    d1_img_output_pkg *img_lr_pkg;
    
    s1_img_output_pkg *img_s1n0_pkg;
    s1_img_output_pkg *img_s1n1_pkg;
    s1_img_output_pkg *img_s1n2_pkg;


    while(cam_n.ok())
    {
        ////////////////////////////////////////////

        // new D1
        if(img_pkg_list_d1.try_pop(img_lr_pkg))
        {
            memcpy(img_left_.data, img_lr_pkg->left_img->data, ce_config_get_cf_img_size());
            left_timestamp = img_lr_pkg->left_img->timestamp;

            double  tmpl =  floor(left_timestamp);
            msg_time.sec=(__time_t) tmpl;
            msg_time.nsec=(__time_t)((left_timestamp - tmpl)* 1e+9);

            left_bridge_.header.stamp = msg_time;
            left_bridge_.header.seq=0;

            memcpy(img_right_.data, img_lr_pkg->right_img->data, ce_config_get_cf_img_size());
            right_timestamp = img_lr_pkg->right_img->timestamp;

            double  tmpr =  floor(right_timestamp);
            msg_time.sec=(__time_t) tmpr;
            msg_time.nsec=(__time_t)((right_timestamp - tmpr)* 1e+9);

            right_bridge_.header.stamp = msg_time;
            right_bridge_.header.seq=0;

            msg_l = left_bridge_.toImageMsg();
            msg_r = right_bridge_.toImageMsg();

            pub_caml_.publish(msg_l);
            pub_camr_.publish(msg_r);

            
            if(ce_config_get_cf_ros_showimage())
            {
                cv::imshow("left",img_left_);
                cv::imshow("right",img_right_);        
                cv::waitKey(1);
            }
            
            delete img_lr_pkg->left_img;
            delete img_lr_pkg->right_img;
            delete img_lr_pkg;
            img_lr_pkg = NULL;
        }

        // new S1N0
        if(img_pkg_list_s1num0.try_pop(img_s1n0_pkg))
        {
            memcpy(img_s1n0_.data, img_s1n0_pkg->s1_img->data, ce_config_get_cf_img_size());
            s1n0_timestamp = img_s1n0_pkg->s1_img->timestamp;

            double  tmps1n0 =  floor(s1n0_timestamp);
            msg_time.sec=(__time_t) tmps1n0;
            msg_time.nsec=(__time_t)((s1n0_timestamp - tmps1n0)* 1e+9);

            s1n0_bridge_.header.stamp = msg_time;
            s1n0_bridge_.header.seq = 0;


            msg_s1n0 = s1n0_bridge_.toImageMsg();

            pub_cams1n0_.publish(msg_s1n0);

            
            if(ce_config_get_cf_ros_showimage())
            {
                cv::imshow("s1n0",img_s1n0_);
                cv::waitKey(1);
            }
            
            delete img_s1n0_pkg->s1_img;
            delete img_s1n0_pkg;
            img_s1n0_pkg = NULL;
        }

        
        // new S1N1
        if(img_pkg_list_s1num1.try_pop(img_s1n1_pkg))
        {
            memcpy(img_s1n1_.data, img_s1n1_pkg->s1_img->data, ce_config_get_cf_img_size());
            s1n1_timestamp = img_s1n1_pkg->s1_img->timestamp;

            double  tmps1n1 =  floor(s1n1_timestamp);
            msg_time.sec=(__time_t) tmps1n1;
            msg_time.nsec=(__time_t)((s1n1_timestamp - tmps1n1)* 1e+9);

            s1n1_bridge_.header.stamp = msg_time;
            s1n1_bridge_.header.seq = 0;


            msg_s1n1 = s1n1_bridge_.toImageMsg();

            pub_cams1n1_.publish(msg_s1n1);

            
            if(ce_config_get_cf_ros_showimage())
            {
                cv::imshow("s1n1",img_s1n1_);
                cv::waitKey(1);
            }
            
            delete img_s1n1_pkg->s1_img;
            delete img_s1n1_pkg;
            img_s1n1_pkg = NULL;
        }


        // new S1N2
        if(img_pkg_list_s1num2.try_pop(img_s1n2_pkg))
        {
            memcpy(img_s1n2_.data, img_s1n2_pkg->s1_img->data, ce_config_get_cf_img_size());
            s1n2_timestamp = img_s1n2_pkg->s1_img->timestamp;

            double  tmps1n2 =  floor(s1n2_timestamp);
            msg_time.sec=(__time_t) tmps1n2;
            msg_time.nsec=(__time_t)((s1n2_timestamp - tmps1n2)* 1e+9);

            s1n2_bridge_.header.stamp = msg_time;
            s1n2_bridge_.header.seq = 0;


            msg_s1n2 = s1n2_bridge_.toImageMsg();

            pub_cams1n2_.publish(msg_s1n2);

            
            if(ce_config_get_cf_ros_showimage())
            {
                cv::imshow("s1n2",img_s1n2_);
                cv::waitKey(1);
            }
            
            delete img_s1n2_pkg->s1_img;
            delete img_s1n2_pkg;
            img_s1n2_pkg = NULL;
        }

        loop_rate.sleep();

        //ros::spinOnce();
    }
}

void CoolEyeCamD1:: imu_data_stream( )
{
    ros::NodeHandle imu_n;
    sensor_msgs::Imu imu_msg;
    ros::Time imu_time;

    ros::Rate loop_rate(2000);

    icm20689_pkg *ticm20689_pkg;
    while(imu_n.ok())
    {
        if(!icm20689_pkg_list.try_pop(ticm20689_pkg))
        {
            loop_rate.sleep();
            continue;
        }


        double  tmp =  floor(ticm20689_pkg->timestamp);
        imu_time.sec=(__time_t) tmp;
        imu_time.nsec=(__time_t)((ticm20689_pkg->timestamp - tmp)* 1e+9);

        imu_msg.header.frame_id = "/imu";
        imu_msg.header.stamp = imu_time;
        imu_msg.header.seq = 0;
        imu_msg.linear_acceleration.x = ticm20689_pkg->ax;
        imu_msg.linear_acceleration.y = ticm20689_pkg->ay;
        imu_msg.linear_acceleration.z = ticm20689_pkg->az;
        
        //imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[0] = 0;
        imu_msg.linear_acceleration_covariance[1] = 0;
        imu_msg.linear_acceleration_covariance[2] = 0;

        imu_msg.linear_acceleration_covariance[3] = 0;
        //imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[4] = 0;
        imu_msg.linear_acceleration_covariance[5] = 0;

        imu_msg.linear_acceleration_covariance[6] = 0;
        imu_msg.linear_acceleration_covariance[7] = 0;
        //imu_msg.linear_acceleration_covariance[8] = 0.04;
        imu_msg.linear_acceleration_covariance[8] = 0;
        
        imu_msg.angular_velocity.x = 3.1415926f * ticm20689_pkg->rx / 180.0f;
        imu_msg.angular_velocity.y = 3.1415926f * ticm20689_pkg->ry / 180.0f;
        imu_msg.angular_velocity.z = 3.1415926f * ticm20689_pkg->rz / 180.0f;
        
        //imu_msg.angular_velocity_covariance[0] = 0.02;
        imu_msg.angular_velocity_covariance[0] = 0;
        imu_msg.angular_velocity_covariance[1] = 0;
        imu_msg.angular_velocity_covariance[2] = 0;

        imu_msg.angular_velocity_covariance[3] = 0;
        //imu_msg.angular_velocity_covariance[4] = 0.02;
        imu_msg.angular_velocity_covariance[4] = 0;
        imu_msg.angular_velocity_covariance[5] = 0;

        imu_msg.angular_velocity_covariance[6] = 0;
        imu_msg.angular_velocity_covariance[7] = 0;
        //imu_msg.angular_velocity_covariance[8] = 0.02;
        imu_msg.angular_velocity_covariance[8] = 0;
        
//         imu_msg.orientation.w = ticm20689_pkg->qw;
//         imu_msg.orientation.x = ticm20689_pkg->qx;
//         imu_msg.orientation.y = ticm20689_pkg->qy;
//         imu_msg.orientation.z = ticm20689_pkg->qz;

        pub_imu_.publish(imu_msg);

        delete ticm20689_pkg;
    }
}

void CoolEyeCamD1::exectu()
{
    cam_thread_ = new boost::thread(boost::bind(&CoolEyeCamD1::img_data_stream, this));
    imu_thread_ = new boost::thread(boost::bind(&CoolEyeCamD1::imu_data_stream, this));

    cam_thread_->join();
    imu_thread_->join();
}


CoolEyeCamD1::~CoolEyeCamD1()
{
}


