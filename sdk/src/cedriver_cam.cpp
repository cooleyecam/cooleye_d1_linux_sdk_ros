#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>
#include <linux/types.h>
#include <algorithm>
#include <fstream>

#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <string>

#include "logmsg.h"
#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "threadsafe_queue.h"
#include "cedriver_config.h"


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"


#define LOG printf

#define CE_CAM_SERIAL_FILE_NAME "../config/serial.txt"

libusb_device_handle *pcaml_handle;
libusb_device_handle *pcamr_handle;

pthread_t ce_camd1l_capture_thread = 0;
pthread_t ce_camd1r_capture_thread = 0;
bool ce_cam_capture_stop_run=false;

pthread_t ce_cam_showimg_thread = 0;
bool ce_cam_showimg_stop_run = false;

pthread_t ce_cam_preprocess_thread = 0;
bool ce_cam_preprocess_stop_run = false;


threadsafe_queue<img_pkg *> img_pkg_left_list;
threadsafe_queue<img_pkg *> img_pkg_right_list;
threadsafe_queue<d1_img_output_pkg *> img_pkg_list_d1;

int ce_cam_read_error_flag_left = false;
int ce_cam_read_error_flag_right = false;

bool ce_cam_rst_flag_left = false;
bool ce_cam_rst_flag_right = false;

extern int g_nCtrl;
int g_nForceWriteFlag;
int g_nCamSerial_D = 1;
int g_nCamSerial_S = 1;

static libusb_device_handle* ce_cam_get_cam_handle(int cam_num)
{
    if(CAMD1_LEFT == cam_num)
        return pcaml_handle;
    else if(CAMD1_RIGHT == cam_num)
        return pcamr_handle;
    else if(CAMS1_MIN <= cam_num && CAMS1_MAX >= cam_num)
        return pcaml_handle;
    else
    {
        LOG("celog: Wrong cam number!\r\n");
        LOG("celog: cam_num = %d \r\n",cam_num);
        return NULL;
    }
}

static int ce_cam_ctrl_camera(int cam_num, unsigned char instruction)
{
    unsigned char buf[1];
    int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num),RT_H2D,instruction,0,0,buf,0,1000);
    if(r != 0)
    {
        LOG("celog: cam%d,i:0x%02X,r:%d\r\n",cam_num,instruction,r);
    }

    return r;
}

static int ce_cam_i2c_read(int cam_num, unsigned char reg,int* value)
{
    unsigned char buf[3];
    int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num),RT_D2H,CAM_I2C_R,(CAM_I2C_ADDR<<8)+reg,0,buf,3,1000);
    if(r != 3)
    {
        LOG("celog: cam%d,i:0x%02X,r:%d\r\n",cam_num,CAM_I2C_R,r);
        return r;
    }
    if(buf[0] != SUCCESS)
    {
        LOG("celog: I2C Read failed. Cam:%d, Addr:0x%02X, Reg:0x%02X, I2C_return:%d\r\n",cam_num,CAM_I2C_ADDR,reg,buf[0]);
        return r;
    }
    *value = buf[1]+(buf[2]<<8);
    return r;
}

static int ce_cam_i2c_write(int cam_num, unsigned char reg,int value)
{
    unsigned char buf[1];
    int  ret = SUCCESS;
    int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num),RT_D2H,CAM_I2C_W,(CAM_I2C_ADDR<<8)+reg,value,buf,1,1000);
    if(r != 1)
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_i2c_write error cam%d, i:0x%02X, r:%d, point:%p\r\n",cam_num,CAM_I2C_W,r, ce_cam_get_cam_handle(cam_num));
        ret = ERROR;
        return ret;
    }

    if(buf[0] != SUCCESS)
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_i2c_write error Cam:%d, Addr:0x%02X, Reg:0x%02X, Write:0x%04X, I2C_return:%d\r\n",cam_num,CAM_I2C_ADDR,reg,value,buf[0]);
        ret = ERROR;
        return ret;
    }
    return ret;
}

static int ce_cam_i2c_readrom(int cam_num, unsigned char *buf)
{
    int ret = SUCCESS;
    int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num), RT_D2H, CAM_DEVICEID, CE_DEVICEID_MEMORY_ADDR, 0, buf, CE_DEVICEID_LEN, 1000);
    if(r != CE_DEVICEID_LEN)
    {
        LOG("celog:ce_cam_i2c_readrom error! cam%d, reqId:0x%X, ret:%d\r\n", cam_num, 0xA9, r);
        ret = ERROR;
        return ret;
    }

#if 0
    LOG("read cam %s deviceid: ", CE_GET_CAM_L_R_STRING(cam_num));
    for(int i=0; i<CE_DEVICEID_LEN; i++)
    {
        LOG("%X ",buf[i]);
    }

    LOG("\r\n");
#endif

    return ret;
}


static int ce_cam_i2c_writerom(int cam_num, unsigned char* buf)
{
    int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num), RT_H2D, CAM_DEVICEID, CE_DEVICEID_MEMORY_ADDR, 0, buf, CE_DEVICEID_LEN, 1000);
    if(r != CE_DEVICEID_LEN)
    {
        LOG("celog:ce_cam_i2c_writerom error! cam%d, reqId:0x%X, ret:%d\r\n", cam_num, 0xA9, r);
        return ERROR;
    }

    return SUCCESS;
}

static int ce_cam_set_af_mode(int camlr)
{
    if(CAMD1_LEFT == camlr)
        ce_cam_i2c_write(camlr,0xAF,0x0303);
    else if(CAMD1_RIGHT == camlr)
        ce_cam_i2c_write(camlr,0xAF,0x0303);
    else
    {
        LOG("celog: Wrong cam number!\r\n");
        return ERROR;
    }

    return SUCCESS;
}




static int ce_cam_sync_rst(int camlr)
{
    if(CAMD1_LEFT == camlr)
        ce_cam_rst_flag_left = true;
    else if(CAMD1_RIGHT == camlr)
        ce_cam_rst_flag_right = true;
    else
    {
        LOG("celog: Wrong cam number!\r\n");
        return ERROR;
    }

    while( (!ce_cam_rst_flag_left)||(!ce_cam_rst_flag_right));


    usleep(100000);

    ce_cam_i2c_write(camlr,0x0C,0x0001);
    ce_cam_ctrl_camera(camlr,STANDBY_SHORT);

    if(CAMD1_LEFT == camlr)
        ce_cam_rst_flag_left = false;
    else if(CAMD1_RIGHT == camlr)
        ce_cam_rst_flag_right = false;

    return SUCCESS;
}


static int ce_cam_sync_rst_bulk(int camlr)
{


    if(CAMD1_LEFT == camlr)
        ce_cam_rst_flag_left = true;
    else if(CAMD1_RIGHT == camlr)
        ce_cam_rst_flag_right = true;
    else
    {
        LOG("celog: Wrong cam number!\r\n");
        return ERROR;
    }



    while( (!ce_cam_rst_flag_left)||(!ce_cam_rst_flag_right));


    usleep(1000);

    ce_cam_i2c_write(camlr,0x0C,0x0001);
    ce_cam_ctrl_camera(camlr,STANDBY_SHORT);

    if(CAMD1_LEFT == camlr)
        ce_cam_rst_flag_left = false;
    else if(CAMD1_RIGHT == camlr)
        ce_cam_rst_flag_right = false;

    return SUCCESS;
}



static void ce_cam_set_mt9v034_config_default(int camlr)
{

    usleep(10000);
    ce_cam_i2c_write(camlr,0x07,0x0188);

    // CONTEXT A
    ce_cam_i2c_write(camlr,0x04,0x02F0);    // 720X480
    ce_cam_i2c_write(camlr,0x03,0x01E0);
    ce_cam_i2c_write(camlr,0x05,0x0106);    // HB
    ce_cam_i2c_write(camlr,0x06,0x024B);    // VB

    ce_cam_i2c_write(camlr,0x0D,0x0300);

    ce_cam_i2c_write(camlr,0x01,0x0001);
    ce_cam_i2c_write(camlr,0x02,0x0004);
    ce_cam_i2c_write(camlr,0x08,0x01BB);
    ce_cam_i2c_write(camlr,0x09,0x01D9);
    ce_cam_i2c_write(camlr,0x0A,0x0164);

    // CONTEXT B
    ce_cam_i2c_write(camlr,0xCC,0x02F0);
    ce_cam_i2c_write(camlr,0xCB,0x01E0);
    ce_cam_i2c_write(camlr,0xCD,0x0106);
    ce_cam_i2c_write(camlr,0xCE,0x024B);

    ce_cam_i2c_write(camlr,0x0E,0x0300);

    ce_cam_i2c_write(camlr,0xC9,0x0001);
    ce_cam_i2c_write(camlr,0xCA,0x0004);
    ce_cam_i2c_write(camlr,0xCF,0x01BB);
    ce_cam_i2c_write(camlr,0xD0,0x01D9);
    ce_cam_i2c_write(camlr,0xD1,0x0164);



    ce_cam_i2c_write(camlr,0x70,0x0303);

    ce_cam_set_af_mode(camlr);

    ce_cam_i2c_write(camlr,0x0F,0x0000);
    ce_cam_i2c_write(camlr,0xAC,0x0001);
    ce_cam_i2c_write(camlr,0xAD,0x01E0);
    ce_cam_i2c_write(camlr,0xAB,0x0040);
    ce_cam_i2c_write(camlr,0xB0,0xFFFF);
    ce_cam_i2c_write(camlr,0xA5,0x0037);
    ce_cam_i2c_write(camlr,0x1C,0x0202);
    ce_cam_i2c_write(camlr,0x7F,0x0000);
    ce_cam_i2c_write(camlr,0xA6,0x0001);
    ce_cam_i2c_write(camlr,0xA8,0x0000);
    ce_cam_i2c_write(camlr,0xA9,0x0000);
    ce_cam_i2c_write(camlr,0xAA,0x0000);


    ce_cam_i2c_write(camlr,0x34,0x0003);
    ce_cam_i2c_write(camlr,0x3c,0x0003);
    ce_cam_i2c_write(camlr,0x2c,0x0004);


    ce_cam_i2c_write(camlr,0x25,0x0020);
    ce_cam_i2c_write(camlr,0xC2,0x0840);

    ce_cam_i2c_write(camlr,0x0C,0x0001);

    ce_cam_i2c_write(camlr,0x47,0x0080);
    ce_cam_i2c_write(camlr,0x48,0x0000);
    ce_cam_i2c_write(camlr,0x4C,0x0002);

    ce_cam_i2c_write(camlr,0x35,0x0028);
    ce_cam_i2c_write(camlr,0x36,0x0028);

    ce_cam_i2c_write(camlr,0xA5,0x0030);
    ce_cam_i2c_write(camlr,0xAD,0x01E0);
    ce_cam_i2c_write(camlr,0xAB,0x0040);

//     ce_cam_i2c_write(camlr,0x72,0x0010);
//     /*recommended register setting and performance impact   PDF-page14*/
//
//     ce_cam_i2c_write(camlr,0x20,0x03C7);
//     ce_cam_i2c_write(camlr,0x24,0x001B);
//     ce_cam_i2c_write(camlr,0x2B,0x0003);
//     ce_cam_i2c_write(camlr,0x2F,0x0003);


}




static void ce_cam_set_mt9v034_fps(int camlr)
{
    ce_cam_i2c_write(camlr,0x05,ce_config_get_cf_img_HB());
    ce_cam_i2c_write(camlr,0x06,ce_config_get_cf_img_VB());
}


static void ce_cam_set_mt9v034_EG_mode(int camlr)
{

   switch (ce_config_get_cf_cam_EG_mode())
    {
    case 0:
        ce_cam_i2c_write(camlr,0xAF,0x00);      //AEC
        ce_cam_i2c_write(camlr,0x0B,ce_config_get_cf_cam_man_exp());   //Exposure Time
        ce_cam_i2c_write(camlr,0x35,ce_config_get_cf_cam_man_gain());  //Gain
        break;
    case 1:
        ce_cam_i2c_write(camlr,0xA5,ce_config_get_cf_cam_auto_EG_des());
        ce_cam_i2c_write(camlr,0xA6,0x01);
        ce_cam_i2c_write(camlr,0xAC,ce_config_get_cf_cam_auto_EG_bottom());
        ce_cam_i2c_write(camlr,0xAD,ce_config_get_cf_cam_auto_EG_top());
        ce_cam_i2c_write(camlr,0xAE,2);
        break;
    case 2:
        ce_cam_i2c_write(camlr,0xAF,0x01);
        ce_cam_i2c_write(camlr,0xA5,ce_config_get_cf_cam_auto_EG_des());
        ce_cam_i2c_write(camlr,0xA6,0x01);
        ce_cam_i2c_write(camlr,0xA8,0x00);
        ce_cam_i2c_write(camlr,0xAC,ce_config_get_cf_cam_auto_E_man_G_Ebottom());
        ce_cam_i2c_write(camlr,0xAD,ce_config_get_cf_cam_auto_E_man_G_Etop());
        ce_cam_i2c_write(camlr,0xAE,2);
        ce_cam_i2c_write(camlr,0x35,ce_config_get_cf_cam_auto_E_man_G());
        break;
    case 3:

        break;
    case 4:
        ce_cam_i2c_write(camlr,0xA6,ce_config_get_cf_cam_agc_aec_skip_frame());
        ce_cam_i2c_write(camlr,0xA8,2);
        ce_cam_i2c_write(camlr,0xA9,ce_config_get_cf_cam_agc_aec_skip_frame());
        ce_cam_i2c_write(camlr,0xAA,2);
        break;
    default:
        break;
    }
}


static void *ce_cam_capture(void *pUserPara)
{
    int camlr = *(int *)pUserPara;
    usleep(1000);
    ce_cam_set_mt9v034_config_default(camlr);

    ce_cam_set_mt9v034_fps(camlr);

    ce_cam_set_mt9v034_EG_mode(camlr);

    ce_cam_ctrl_camera(camlr,SET_MCLK_48MHz);

    usleep(1000);

    ce_cam_sync_rst(camlr);

    struct timeval cap_systime;

    int r,transferred = 0;
    unsigned char pass;

    img_pkg *timg_pkg;
    threadsafe_queue<img_pkg *> *tlist;
    int error_count = 0;

    if(CAMD1_LEFT == camlr)
    {
        tlist = &img_pkg_left_list;
        ce_cam_read_error_flag_left = false;
    }
    else if(CAMD1_RIGHT == camlr)
    {
        tlist = &img_pkg_right_list;
        ce_cam_read_error_flag_right = false;
    }


    libusb_device_handle *pcam_handle;
    pcam_handle = ce_cam_get_cam_handle(camlr);

    while(!ce_cam_capture_stop_run)
    {
        timg_pkg = new img_pkg;
        if (NULL == timg_pkg)
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture ce_cam_capture alloc memory failure! exit thread!\r\n");
            break;
        }

        memset(timg_pkg, 0, sizeof(img_pkg));
        r = libusb_bulk_transfer(pcam_handle, 0x82, timg_pkg->data, ce_config_get_cf_img_buff_size(), &transferred, 1000);
        gettimeofday(&cap_systime,NULL);
        timg_pkg->timestamp = cap_systime.tv_sec+0.000001*cap_systime.tv_usec-ce_config_get_cf_img_time_offset();

        if(r)
        {
            //LOG("cam %d bulk transfer returned: %d\n",camlr,r);
            error_count++;
            if (error_count >= 1000)
            {
                if(CAMD1_LEFT == camlr)
                {
                    //ce_camd1l_capture_thread = 0;
                    ce_cam_read_error_flag_left = true;
                }
                else if(CAMD1_RIGHT == camlr)
                {
                    //ce_camd1r_capture_thread = 0;
                    ce_cam_read_error_flag_right = true;
                }

                delete timg_pkg;
                timg_pkg = NULL;
                pthread_exit(NULL);
            }
            else
            {
                delete timg_pkg;
                timg_pkg = NULL;
                continue;
            }
        }

        pass = (timg_pkg->data[ce_config_get_cf_img_size()+0]==0xFF
                &&timg_pkg->data[ce_config_get_cf_img_size()+1]==0x00
                &&timg_pkg->data[ce_config_get_cf_img_size()+2]==0xFE
                &&timg_pkg->data[ce_config_get_cf_img_size()+3]==0x01);

        if((pass == 1)&&(ce_cam_rst_flag_left == false)&&(ce_cam_rst_flag_right == false))
        {
            error_count = 0;
            img_pkg *timg_pkg_giveup = NULL;

            if (0 == tlist->push(timg_pkg, timg_pkg_giveup, 30))
            {
                delete timg_pkg_giveup;
            }
        }
        else
        {
            delete timg_pkg;
            timg_pkg = NULL;
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture cam %d bulk transfer check failed: %d\n",camlr,r);

            ce_cam_ctrl_camera(camlr,SET_MCLK_48MHz);
        }
    }

#if 0
    if(CAMD1_LEFT == camlr)
    {
        ce_camd1l_capture_thread = 0;
    }
    else if(CAMD1_RIGHT == camlr)
    {
        ce_camd1r_capture_thread = 0;
    }

    pthread_exit(NULL);
#endif
}

static void ce_cam_get_soft_version(int camlr)
{
    unsigned char ver_buf[30];

    int j = libusb_control_transfer(ce_cam_get_cam_handle(camlr),RT_D2H,GET_THE_SOFT_VISION,0,0,ver_buf,30,1000);
    if(CAMD1_LEFT == camlr)
    {
        LOG("celog: the left cam version :");
    }
    else if(CAMD1_RIGHT == camlr)
    {
        LOG("celog: the right cam version :");
    }


    for(int i = 0; i< j; i++)
    {
        LOG("%c",ver_buf[i]);
    }

    LOG("\r\n");
}

int ce_get_cam_serial_from_log(int *serial_D, int *serial_S)
{
    //std::fstream serialFile(CE_CAM_SERIAL_FILE_NAME, std::fstream::out);
    std::fstream serialFile(CE_CAM_SERIAL_FILE_NAME);
    if(!serialFile)
    {
        std::cerr << "celog: opening file failed -- "<<CE_CAM_SERIAL_FILE_NAME <<std::endl;
        exit(EXIT_FAILURE);
    }

    std::string line;
    while(getline(serialFile, line))
    {
        if(line.at(0)!='#')
        {
            const char *ptr = line.c_str();

            if (0 == strncmp(ptr, "serial_D=", strlen("serial_D=")))
            {
                *serial_D = atoi(ptr+strlen("serial_D="));
            }

            if (0 == strncmp(ptr, "serial_S=", strlen("serial_S=")))
            {
                *serial_S = atoi(ptr+strlen("serial_S="));
            }
        }
    }

    serialFile.close();
    return 0;
}

int ce_set_cam_serial_to_log(int serial_D, int serial_S)
{
    std::ofstream serialFile(CE_CAM_SERIAL_FILE_NAME);
    if(!serialFile )
    {
        std::cerr << "celog: opening file failed -- "<<CE_CAM_SERIAL_FILE_NAME <<std::endl;
        exit(EXIT_FAILURE);
    }

    serialFile<<"serial_D="<<serial_D<<std::endl;
    serialFile<<"serial_S="<<serial_S<<std::endl;

    serialFile.close();
    return 0;
}

int ce_cam_check_deviceid(char *buf, int *serial)
{
    if (buf[0] != 'E' || buf[1] != 'C' || buf[2] != 'A')
        return ERROR;

    char tmpBuf[10] = {0};
    strncpy(tmpBuf, &buf[7], CE_CAM_SERIAL_LEN);

    *serial = atoi(tmpBuf);

    return SUCCESS;
}

int ce_cam_build_deviceid(char *buf, char type, char *subType, int camlr, int camSerial)
{
    memset(buf, 0, CE_DEVICEID_LEN);
    char *cur = buf;

    /*format serial*/
    char serial[10] = {0};
    sprintf(serial, "%05d", camSerial);

    /* class 1: ECA */
    buf[0] = 'E';//E
    buf[1] = 'C';//C
    buf[2] = 'A';//A

    /* class 2: type */
    buf[3] = type;

    /* class 3: sub type */
    strncat(buf, subType, 3);

    /* class 4: Serial Number */
    strncat(buf, serial, CE_CAM_SERIAL_LEN);

    /* class 5: Serial Number */
    int len = strlen(buf);
    if (CAMD1_LEFT == camlr)
        buf[len] = 'L';
    else if (CAMD1_RIGHT == camlr)
        buf[len] = 'R';
    else
        buf[len] = '0';

    return 0;
}

int ce_cam_write_deviceid_S1()
{
    int ret;
    int usbNumber;
    bool newCamFalg = true;
    ce_get_cam_serial_from_log(&g_nCamSerial_D, &g_nCamSerial_S);

    g_nCtrl = 1;
    while(g_nCtrl)
    {
        pcaml_handle = NULL;
        pcamr_handle = NULL;
        ret = SUCCESS;

        usbNumber = ce_usb_open();
        if(usbNumber < 1)
        {
            ret = ERROR;
        }
        else
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "发现ＵＳＢ设备, 数量: %d\r\n", usbNumber);
        }

        libusb_device_handle *pusb_handle;
        for(int i = 0; i < usbNumber; i++)
        {
            unsigned char buf = 0;
            pusb_handle = ce_usb_gethandle(i);
            int r_num = libusb_control_transfer(pusb_handle,RT_D2H,GET_CAM_LR,0,0,&buf,1,1000);
            if(r_num != 1)
            {
                //LOG("celog: Get the device LR addr failed\r\n");
                ret= ERROR;
            }
            else if (CAMS1_MIN <= buf && CAMS1_MAX >= buf)
            {
                pcaml_handle = pusb_handle;
            }
            else
            {
                ret= ERROR;
            }
        }


        if(ERROR == ret)
        {
            LOG("未检测到摄像头,请插入ＵＳＢ设备\r\n");
        }
        else
        {
            int temp = 0;
            static int caml_addr = CAMS1_00;

#if 1
            // pre check cam left deviceid
            char readbufL[16] = {0};
            bool readbufL_flag = false;
            int readbufL_serial = 0;

            if (SUCCESS == ce_cam_i2c_readrom(caml_addr, (unsigned char *)readbufL))
            {
                if (SUCCESS == ce_cam_check_deviceid(readbufL, &readbufL_serial))
                {
                    readbufL_flag = true;
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "该摄像头已有编号: %s\r\n", readbufL);
                }
                else
                {
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_check_deviceid fail! readbufL=[0x%x 0x%x 0x%x] \r\n", readbufL[0], readbufL[1], readbufL[2]);
                }
            }
            else
            {
                LOG("read left cam deviceid fail!\r\n");
            }

            if ((readbufL_flag && !g_nForceWriteFlag) ||    //已写入，且非强制写入模式
                (readbufL_flag && readbufL_serial == g_nCamSerial_S - 1)) //前一次刚写入
            {
                 WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "该摄像头已有设备编号,请换其他摄像头\r\n");
            }
            else if(readbufL_flag && readbufL_serial == g_nCamSerial_S) //g_nCamSerial_S = 1,readbufL_serial = 1
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "该摄像头已有设备编号,请换其他摄像头\r\n");

                g_nCamSerial_S++;
                if (g_nCamSerial_S > 99999)
                    g_nCamSerial_S = 1;

                ce_set_cam_serial_to_log(g_nCamSerial_D, g_nCamSerial_S);
            }
            else
            {
                //printf("------------- %d %d %d %d %d %d\r\n", readbufL_flag, readbufR_flag, readbufL_serial, readbufR_serial, g_nCamSerial_D, g_nForceWriteFlag);
                char writebufL[16] = {0};
                bool writebufL_flag = false;

                /* write left cam deviceid */
                ce_cam_build_deviceid(writebufL, CE_CAM_TYPE_S, CE_CAM_SUB_TYPE_S1_V2P0, CAMS1_00, g_nCamSerial_S);
                if(SUCCESS == ce_cam_i2c_writerom(caml_addr, (unsigned char *)writebufL))
                {
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "重新写入左摄像头编号: %s\r\n", writebufL);
                    writebufL_flag = true;
                }

                if (writebufL_flag)
                {
                    g_nCamSerial_S++;
                    if (g_nCamSerial_S > 99999)
                        g_nCamSerial_S = 1;

                    ce_set_cam_serial_to_log(g_nCamSerial_D, g_nCamSerial_S);
                }
            }
            LOG("\r\n");
#endif
        }

        sleep(1);
        ce_usb_close();
    }

    return SUCCESS;
}

int ce_cam_write_deviceid_D1()
{
    int ret;
    int usbNumber;
    bool newCamFalg = true;
    ce_get_cam_serial_from_log(&g_nCamSerial_D, &g_nCamSerial_S);

    g_nCtrl = 1;
    while(g_nCtrl)
    {
        pcaml_handle = NULL;
        pcamr_handle = NULL;
        ret = SUCCESS;

        usbNumber = ce_usb_open();
        if(usbNumber < 1)
        {
            ret = ERROR;
        }
        else
        {
             WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "发现ＵＳＢ设备, number = %d\r\n", usbNumber);
        }

        libusb_device_handle *pusb_handle;
        for(int i = 0; i < usbNumber; i++)
        {
            unsigned char buf = 0;
            pusb_handle = ce_usb_gethandle(i);
            int r_num = libusb_control_transfer(pusb_handle,RT_D2H,GET_CAM_LR,0,0,&buf,1,1000);
            if(r_num != 1)
            {
                //LOG("celog: Get the device LR addr failed\r\n");
                ret= ERROR;
            }
            else
            {
                if(CAMD1_LEFT == buf)
                {
                    pcaml_handle = pusb_handle;

                }
                else if(CAMD1_RIGHT == buf)
                {
                    pcamr_handle = pusb_handle;

                }
            }
        }

        if(pcaml_handle==NULL)
        {
            ret= ERROR_LOST_LEFT_CAM;
             WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "未检测到左摄像头\r\n");
        }

        if(pcamr_handle==NULL)
        {
            ret= ERROR_LOST_RIGHT_CAM;
             WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "未检测到右摄像头\r\n");
        }

        if(ERROR == ret)
        {
             WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "未检测到摄像头,请插入ＵＳＢ设备\r\n");
        }
        else if (SUCCESS != ret)
        {
        }
        else
        {
            int temp=0;
            static int caml_addr = CAMD1_LEFT;
            static int camr_addr = CAMD1_RIGHT;

#if 1
            // pre check cam left deviceid
            char readbufL[16] = {0};
            char readbufR[16] = {0};
            bool readbufL_flag = false;
            bool readbufR_flag = false;
            int readbufL_serial = 0;
            int readbufR_serial = 0;
            //ce_cam_i2c_readrom(caml_addr, (unsigned char *)readbufL);
            if (SUCCESS == ce_cam_i2c_readrom(caml_addr, (unsigned char *)readbufL))
            {
                if (SUCCESS == ce_cam_check_deviceid(readbufL, &readbufL_serial))
                {
                    readbufL_flag = true;
                }
                else
                {
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_check_deviceid fail! readbufL = %s\r\n", readbufL);
                }
            }
            else
            {
                 WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "read left cam deviceid fail!\r\n");
            }

            // pre check cam right deviceid
            if (SUCCESS == ce_cam_i2c_readrom(camr_addr, (unsigned char *)readbufR))
            {
                if (SUCCESS == ce_cam_check_deviceid(readbufR, &readbufR_serial))
                {
                    readbufR_flag = true;
                }
                else
                {
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_check_deviceid fail! readbufR = %s\r\n", readbufR);
                }
            }
            else
            {
                 WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "read right cam deviceid fail!\r\n");
            }

            if (readbufL_flag && readbufR_flag)
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "左摄像头已有编号: %s\r\n", readbufL);
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "右摄像头已有编号: %s\r\n", readbufR);

                if (readbufL_serial != readbufR_serial)
                {
                    WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "左右摄像头编号不一致，需要重新写入... \r\n");
                    readbufL_flag = false;
                    readbufR_flag = false;
                }
            }

            if ((readbufL_flag && readbufR_flag && !g_nForceWriteFlag) ||
                (readbufL_flag && readbufR_flag && (readbufL_serial == g_nCamSerial_D - 1)))
            {
                 WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "该摄像头已有设备编号，请换其他摄像头\r\n");
            }
            else if (readbufL_flag && readbufR_flag && (readbufL_serial == g_nCamSerial_D))// g_nCamSerial_D=1, readbufL_serial=1
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "该摄像头已有设备编号,请换其他摄像头\r\n");
                g_nCamSerial_D++;
                if (g_nCamSerial_D > 99999)
                    g_nCamSerial_D = 1;

                ce_set_cam_serial_to_log(g_nCamSerial_D, g_nCamSerial_S);
            }
            else
            {
                //printf("------------- %d %d %d %d %d %d\r\n", readbufL_flag, readbufR_flag, readbufL_serial, readbufR_serial, g_nCamSerial_D, g_nForceWriteFlag);
                char writebufL[16] = {0};
                char writebufR[16] = {0};
                bool writebufL_flag = false;
                bool writebufR_flag = false;

                /* write left cam deviceid */
                ce_cam_build_deviceid(writebufL, CE_CAM_TYPE_D, CE_CAM_SUB_TYPE_D1_V1P6, CAMD1_LEFT, g_nCamSerial_D);
                if(SUCCESS == ce_cam_i2c_writerom(caml_addr, (unsigned char *)writebufL))
                {
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "重新写入左摄像头编号: %s\r\n", writebufL);
                    writebufL_flag = true;

                }

                /* write right cam deviceid */
                ce_cam_build_deviceid(writebufR, CE_CAM_TYPE_D, CE_CAM_SUB_TYPE_D1_V1P6, CAMD1_RIGHT, g_nCamSerial_D);
                if(SUCCESS == ce_cam_i2c_writerom(camr_addr, (unsigned char *)writebufR))
                {
                     WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "重新写入右摄像头编号: %s\r\n", writebufR);
                    writebufR_flag = true;
                }

                if (writebufL_flag && writebufR_flag)
                {
                    g_nCamSerial_D++;
                    if (g_nCamSerial_D > 99999)
                        g_nCamSerial_D = 1;

                    ce_set_cam_serial_to_log(g_nCamSerial_D, g_nCamSerial_S);
                }
            }
             WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "\r\n");
#endif
        }

        sleep(1);
        ce_usb_close();
    }

    return SUCCESS;
}

int ce_cam_capture_init()
{
    int ret_val = SUCCESS;
    int r;

    int count = 0;
    while(g_nCtrl)
    {
        sleep(1);
        ret_val = SUCCESS;
        pcaml_handle = NULL;
        pcamr_handle = NULL;
        r = ce_usb_open();
        if(r<1)
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init No cameras found! r = %d, count = %d\r\n",r, count++);
            continue;
        }
        else
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_cam_capture_init Number of device of interest found: %d\r\n",r);
        }

        libusb_device_handle *pusb_handle;
        for(int i=0; i<r; i++)
        {
            unsigned char buf = 0;
            pusb_handle = ce_usb_gethandle(i);
            int r_num = libusb_control_transfer(pusb_handle,RT_D2H,GET_CAM_LR,0,0,&buf,1,1000);
            if(r_num != 1)
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init: Get the device LR addr failed\r\n");
            }
            else
            {
                if(CAMD1_LEFT == buf)
                {
                    pcaml_handle = pusb_handle;
                    WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_cam_capture_init pcaml_handle = %p\r\n", pcaml_handle);

                }
                else if(CAMD1_RIGHT == buf)
                {
                    pcamr_handle = pusb_handle;
                    WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_cam_capture_init pcamr_handle = %p\r\n", pcamr_handle);

                }
            }
        }

        if(ce_config_get_cf_cam_mode() & CAMD1_LEFT_ENABLE)
        {
            if(pcaml_handle==NULL)
                ret_val= ERROR;
        }

        if(ce_config_get_cf_cam_mode() & CAMD1_RIGHT_ENABLE)
        {
            if(pcamr_handle==NULL)
                ret_val= ERROR;
        }

        if(ERROR == ret_val)
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init lost a cam left or right \r\n");
            //return ERROR;
            ce_usb_close();
            continue;
        }
        else
        {
            int temp=0;
            static int caml_addr = CAMD1_LEFT;
            static int camr_addr = CAMD1_RIGHT;

            ce_cam_get_soft_version(caml_addr);
            ce_cam_get_soft_version(camr_addr);

            ce_cam_read_error_flag_left = false;
            ce_cam_read_error_flag_right = false;
            if(ce_config_get_cf_cam_mode()& CAMD1_LEFT_ENABLE)
            {
                temp = pthread_create(&ce_camd1l_capture_thread, NULL, ce_cam_capture, &caml_addr);
                if(temp)
                {
                    WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init create thread failed !\r\n");
                    ce_cam_read_error_flag_left = true;
                    //return ERROR;
                }
            }

            if(ce_config_get_cf_cam_mode() & CAMD1_RIGHT_ENABLE)
            {
                temp = pthread_create(&ce_camd1r_capture_thread, NULL, ce_cam_capture, &camr_addr);
                if(temp)
                {
                    WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init create thread failed !\r\n");
                    ce_cam_read_error_flag_right = true;
                    //return ERROR;
                }
            }

            while (g_nCtrl && (!ce_cam_read_error_flag_left || !ce_cam_read_error_flag_right))
            {
                usleep(1000);
            }
        }

        ce_usb_close();
    }

    WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init finish\r\n");
    return SUCCESS;
}

void ce_cam_capture_close()
{
    ce_cam_capture_stop_run=true;
    sleep(1);
    if(ce_camd1l_capture_thread !=0)
    {
        pthread_join(ce_camd1l_capture_thread,NULL);
    }
    if(ce_camd1r_capture_thread !=0)
    {
        pthread_join(ce_camd1r_capture_thread,NULL);
    }
    ce_usb_close();
}

static void* ce_cam_showimg(void *)
{
    cv::Mat  img_left(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);

    d1_img_output_pkg *img_lr_pkg;

    while(!ce_cam_showimg_stop_run)
    {
        if(!img_pkg_list_d1.try_pop(img_lr_pkg))
        {
                usleep(1000);
                continue;
        }


        memcpy(img_left.data, img_lr_pkg->left_img->data, ce_config_get_cf_img_size());
        //cv::imshow("left",img_left);
//        std::cout << "left tamps:" << std::setprecision(15) << img_lr_pkg->left_img->timestamp << std::endl;

        memcpy(img_right.data,img_lr_pkg->right_img->data,ce_config_get_cf_img_size());
        //cv::imshow("right",img_right);
//        std::cout << "right tamps:" << std::setprecision(15) << img_lr_pkg->right_img->timestamp << std::endl;



        cv::Mat result(img_left.rows,
                   img_left.cols + img_right.cols,
                   img_left.type());



        img_left.colRange( 0, img_left.cols).copyTo(result.colRange(0, img_left.cols));

        img_right.colRange( 0, img_right.cols).copyTo(result.colRange(img_left.cols, result.cols));
        cv::circle(result, cv::Point(376,240),10, cv::Scalar(0,0,255));
        cv::circle(result, cv::Point(1128,240),10, cv::Scalar(0,0,255));
    cv::imshow("result",result);

        cv::waitKey(1);

        delete img_lr_pkg->left_img;
        delete img_lr_pkg->right_img;
        delete img_lr_pkg;
    }

    ce_cam_showimg_thread = 0;
    pthread_exit(NULL);
}

int ce_cam_showimg_init()
{
    int temp = pthread_create(&ce_cam_showimg_thread, NULL, ce_cam_showimg, NULL);
    if(temp)
    {
        LOG("celog: Failed to create thread show image \r\n");
        return ERROR;

    }
    return SUCCESS;
}

void ce_cam_showimg_close()
{
    ce_cam_showimg_stop_run = true;
    usleep(100);

    if(ce_cam_showimg_thread != 0)
    {
        pthread_join(ce_cam_showimg_thread,NULL);
    }
}

static void* ce_cam_preprocess(void *)
{


    cv::Mat img_left(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);

    cv::Mat img_left_remap(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat img_right_remap(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);

    cv::Mat M1, D1, M2, D2;
    cv::Mat R, T, R1, P1, R2, P2 ,Q;

    cv::Mat l_remapx,l_remapy,r_remapx,r_remapy;

    cv::FileStorage fs(ce_config_get_cf_cam_intrinsics(), cv::FileStorage::READ);
    if(!fs.isOpened())
    {

        printf("Failed to open file intrinsic_filename \n");
    }

    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(ce_config_get_cf_cam_extrinsics(), cv::FileStorage::READ);
    if(!fs.isOpened())
    {

        printf("Failed to open file extrinsics_filename \n");

    }

    fs["R"] >> R;
    fs["T"] >> T;

    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;


    cv::fisheye::initUndistortRectifyMap(M1, D1, R1, P1, img_left.size(), CV_16SC2, l_remapx, l_remapy);
    cv::fisheye::initUndistortRectifyMap(M2, D2, R2, P2, img_right.size(), CV_16SC2, r_remapx, r_remapy);


    img_pkg timg_pkg;

    img_pkg *l_img_pkg = NULL;
    img_pkg *r_img_pkg = NULL;


    while(!ce_cam_preprocess_stop_run)
    {
        if((!img_pkg_left_list.empty()) && (!img_pkg_right_list.empty()))
        {
            img_pkg_left_list.try_front(l_img_pkg);
            img_pkg_right_list.try_front(r_img_pkg);

            double diff_tamps = fabs(l_img_pkg->timestamp - r_img_pkg->timestamp);

            if( diff_tamps > 0.005 )
            {
                std::cout << "celog: something error the image is no sync!\r\n" << std::endl;
                if(l_img_pkg->timestamp < r_img_pkg->timestamp )  // give up the early data
                {
                    img_pkg_left_list.try_pop(l_img_pkg);
                    delete l_img_pkg;
                    l_img_pkg = NULL;
                }
                else
                {
                    img_pkg_right_list.try_pop(r_img_pkg);
                    delete r_img_pkg;
                    r_img_pkg = NULL;
                }
            }
            else
            {
                img_pkg_left_list.try_pop(l_img_pkg);
                img_pkg_right_list.try_pop(r_img_pkg);

                d1_img_output_pkg *t_output_pkg = new d1_img_output_pkg;
                if (NULL == t_output_pkg)
                {
                    LOG("celog: ce_cam_showimg alloc memory failure!\r\n");
                    delete l_img_pkg;
                    delete r_img_pkg;
                    continue;
                }

                t_output_pkg->left_img = l_img_pkg;
                t_output_pkg->right_img = r_img_pkg;

                t_output_pkg->left_img->timestamp = l_img_pkg->timestamp;      // merger the timestamp to left
                t_output_pkg->right_img->timestamp = l_img_pkg->timestamp;


                if(ce_config_get_cf_cam_rectify())
                {
                    memcpy(img_left.data, t_output_pkg->left_img->data, ce_config_get_cf_img_size());
                    memcpy(img_right.data,t_output_pkg->right_img->data,ce_config_get_cf_img_size());

                    remap(img_left, img_left_remap, l_remapx, l_remapy, cv::INTER_LINEAR);
                    remap(img_right, img_right_remap, r_remapx, r_remapy, cv::INTER_LINEAR);

                    memcpy(t_output_pkg->left_img->data, img_left_remap.data, ce_config_get_cf_img_size());
                    memcpy(t_output_pkg->right_img->data, img_right_remap.data, ce_config_get_cf_img_size());
                }

                d1_img_output_pkg *t_pkg_giveup = NULL;

                img_pkg_list_d1.push(t_output_pkg, t_pkg_giveup, 30);
                if (NULL != t_pkg_giveup)
                {
                    delete t_pkg_giveup->left_img;
                    delete t_pkg_giveup->right_img;
                    delete t_pkg_giveup;
                }
            }

        }
        usleep(1000);
    }
    pthread_exit(NULL);
}

int ce_cam_preprocess_init()
{
    int temp = pthread_create(&ce_cam_preprocess_thread, NULL, ce_cam_preprocess, NULL);
    if(temp)
    {
        LOG("celog: Failed to create thread show image \r\n");
        return ERROR;

    }
    return SUCCESS;
}

void ce_cam_preprocess_close()
{
    ce_cam_preprocess_stop_run = true;
    if(ce_cam_preprocess_thread != 0)
    {
        pthread_join(ce_cam_preprocess_thread,NULL);
    }

}
