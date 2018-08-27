#ifndef CEDRIVER_GLOBAL_CONFIG_H
#define CEDRIVER_GLOBAL_CONFIG_H

#include <string>

//#define USING_RING_QUEUE    0
//////////////////////////////////////////////////////////////////////////
//       system define
//////////////////////////////////////////////////////////////////////////

#define SUCCESS     0
#define ERROR       -1


//////////////////////////////////////////////////////////////////////////
//////   CAM D1  USB setting
//////////////////////////////////////////////////////////////////////////

#define PID_CE_D1   0x1003
#define VID_CE_D1   0x04B4

#define MAXDEVICES      2


#define RT_H2D      0x40
#define RT_D2H      0xC0

//////////////////////////////////////////////////////////////////////////
//////   CAM D1  IMU setting
//////////////////////////////////////////////////////////////////////////
#define IMU_FRAME_LEN   16
#define IMU_FIFO_SIZE   50
#define IMU_DELAY_OFFSET    (double)0.00065



#define CMD_HEAD1     0x55
#define CMD_HEAD2     0xAA
#define CMD_IMC20689_START      0x02
#define CMD_IMC20689_SAMPLE_RATE 0x04
#define STD_g   9.80665
//////////////////////////////////////////////////////////////////////////
//////   CAM D1  CAM setting
//////////////////////////////////////////////////////////////////////////
#define CAMD1_SYS_CLKIN 27000000

#define CAMD1_LEFT      0xF1
#define CAMD1_RIGHT     0xF0



#define CAMD1_RESOLUTION_VGA 1
#define CAMD1_RESOLUTION_WVGA 2

#define CAMD1_LEFT_ENABLE   0x01
#define CAMD1_RIGHT_ENABLE   0x02



/*PARAMETER MACRO*/
#define IMG_WIDTH_VGA   640
#define IMG_HEIGHT_VGA  480
#define IMG_SIZE_VGA    (IMG_WIDTH_VGA*IMG_HEIGHT_VGA)
#define IMG_BUF_SIZE_VGA (IMG_SIZE_VGA+0x200)
#define IMG_WIDTH_WVGA  752
#define IMG_HEIGHT_WVGA     480
#define IMG_SIZE_WVGA   (IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA)
#define IMG_BUF_SIZE_WVGA (IMG_SIZE_WVGA+0x200)
#define FRAME_CLUST 54

/*CAMERA CONTROL INSTRUCTION MACRO*/
#define SET_MCLK_48MHz      0xA1
#define SET_MCLK_24MHz      0xA2
#define STANDBY_SHORT       0xA3
#define STANDBY_LONG        0xA4
#define GET_CAM_LR          0xA5

#define GET_THE_SOFT_VISION 0xA6

#define CAM_I2C_R           0xA7
#define CAM_I2C_W           0xA8
#define auto_expo           1
#define EXP_VAL             50
#define GAI_VAL             100

#define CAM_I2C_ADDR        0x5C //0x5C


//////////////////////////////////////////////////////////////////////////////////
//
//      config.txt  struct
//////////////////////////////////////////////////////////////////////////////////

typedef struct
{
    std::string cf_dev_name;             // D1
    std::string cf_dev_version;          // V1.0 reserved
}global_dev_config;

typedef struct
{
    int cf_cam_mode;     // left - 2b'01 / right-2b'10 /sterero-2b'11
    int cf_cam_resolution;   //
    int cf_cam_FPS;      // FPS
    int cf_cam_EG_mode;
    int cf_cam_man_exp;
    int cf_cam_man_gain;
    int cf_cam_auto_EG_top;
    int cf_cam_auto_EG_bottom;
    int cf_cam_auto_EG_des;
    int cf_cam_auto_E_man_G_Etop;
    int cf_cam_auto_E_man_G_Ebottom;
    int cf_cam_auto_E_man_G;
    int cf_cam_agc_aec_skip_frame;
    int cf_cam_rectify;
    std::string cf_cam_intrinsics;
    std::string cf_cam_extrinsics;
}global_cam_config;

typedef struct
{
    std::string cf_imu_uart;
    float cf_imu_icm20689_acc_bias_X;
    float cf_imu_icm20689_acc_bias_Y;
    float cf_imu_icm20689_acc_bias_Z;
    float cf_imu_icm20689_gyro_bias_X;
    float cf_imu_icm20689_gyro_bias_Y;
    float cf_imu_icm20689_gyro_bias_Z;
    float cf_imu_icm20689_acc_T[3][3];
    int cf_imu_icm20689_sample_rate;
}global_imu_config;

typedef struct
{
    int cf_img_width;
    int cf_img_height;
    int cf_img_size;
    int cf_img_buff_size;
    int cf_img_HB;
    int cf_img_VB;
    double cf_img_time_offset;
}global_img_config;


typedef struct
{
    int cf_ros_showimage;
}global_ros_config;


typedef struct
{
    int cf_ste_algorithm;
}global_ste_config;


typedef struct
{
    global_dev_config   gc_dev;
    global_cam_config   gc_cam;
    global_imu_config   gc_imu;
    global_img_config   gc_img;
    global_ros_config   gc_ros;
    global_ste_config   gc_ste;
    
}global_config_d1;




#endif
