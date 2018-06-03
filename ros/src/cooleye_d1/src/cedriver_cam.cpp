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

#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "threadsafe_queue.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cedriver_config.h"

#define LOG         printf


libusb_device_handle *pcaml_handle;
libusb_device_handle *pcamr_handle;

pthread_t ce_camd1l_capture_thread = 0;
pthread_t ce_camd1r_capture_thread = 0;
bool ce_cam_capture_stop_run=false;

pthread_t ce_cam_showimg_thread;
bool ce_cam_showimg_stop_run = false;

pthread_t ce_cam_preprocess_thread;
bool ce_cam_preprocess_stop_run = false;

threadsafe_queue<img_pkg *> img_pkg_left_list;
threadsafe_queue<img_pkg *> img_pkg_right_list;
threadsafe_queue<d1_img_output_pkg *> img_pkg_list_d1;



bool ce_cam_rst_flag_left = false;
bool ce_cam_rst_flag_right = false;


static libusb_device_handle* ce_cam_get_cam_handle(int cam_num)
{
    if(CAMD1_LEFT == cam_num)
        return pcaml_handle;
    else if(CAMD1_RIGHT == cam_num)
        return pcamr_handle;
    else
    {
        LOG("celog: Wrong cam number!\r\n");
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
    int r = libusb_control_transfer(ce_cam_get_cam_handle(cam_num),RT_D2H,CAM_I2C_W,(CAM_I2C_ADDR<<8)+reg,value,buf,1,1000);
    if(r != 1)
    {
        LOG("celog: cam%d,i:0x%02X,r:%d\r\n",cam_num,CAM_I2C_W,r);
        return r;
    }
    if(buf[0] != SUCCESS)
    {
        LOG("celog: I2C Write failed.  Cam:%d, Addr:0x%02X, Reg:0x%02X, Write:0x%04X, I2C_return:%d\r\n",cam_num,CAM_I2C_ADDR,reg,value,buf[0]);
        return r;
    }
    return r;
}

static void ce_cam_set_mt9v034_default(int camlr)
{
    /*every reg you have changed should be set to dfault here;*/
    ce_cam_i2c_write(camlr,0x00,0x1324);
    ce_cam_i2c_write(camlr,0x01,0x0001);
    ce_cam_i2c_write(camlr,0x02,0x0004);
    ce_cam_i2c_write(camlr,0x03,0x01E0);

    ce_cam_i2c_write(camlr,0x04,0x02F0);
    ce_cam_i2c_write(camlr,0x05,0x005E);
    ce_cam_i2c_write(camlr,0x06,0x002D);
    ce_cam_i2c_write(camlr,0x07,0x0388);


    ce_cam_i2c_write(camlr,0x0B,0x01E0);
    ce_cam_i2c_write(camlr,0x35,0x0010);
    ce_cam_i2c_write(camlr,0xA5,0x003A);
    ce_cam_i2c_write(camlr,0xA6,0x0002);

    ce_cam_i2c_write(camlr,0xA8,0x0000);
    ce_cam_i2c_write(camlr,0xA9,0x0002);
    ce_cam_i2c_write(camlr,0xAA,0x0002);
    ce_cam_i2c_write(camlr,0xAB,0x0040);

    ce_cam_i2c_write(camlr,0xAC,0x0001);
    ce_cam_i2c_write(camlr,0xAD,0x01E0);
    ce_cam_i2c_write(camlr,0xAE,0x0014);
    ce_cam_i2c_write(camlr,0xAF,0x0003);

    /*recommended register setting and performance impact   PDF-page14*/
    ce_cam_i2c_write(camlr,0x20,0x03C7);
    ce_cam_i2c_write(camlr,0x24,0x001B);
    ce_cam_i2c_write(camlr,0x2B,0x0003);
    ce_cam_i2c_write(camlr,0x2F,0x0003);
}

static int ce_cam_set_af_mode(int camlr)
{
    if(CAMD1_LEFT == camlr)
        ce_cam_i2c_write(camlr,0x70,0x0303);
    else if(CAMD1_RIGHT == camlr)
        ce_cam_i2c_write(camlr,0x70,0x0000);
      //  ce_cam_i2c_write(camlr,0x70,0x0303);
    else
    {
        LOG("celog: Wrong cam number!\r\n");
        return ERROR;
    }

    return SUCCESS;
}


static int ce_cam_set_master_slave_mode(int camlr)
{
    if(CAMD1_LEFT == camlr)
        ce_cam_i2c_write(camlr,0x07,0x0088);
    else if(CAMD1_RIGHT == camlr)
        ce_cam_i2c_write(camlr,0x07,0x0080);
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
    ce_cam_i2c_write(camlr,0x07,0x0388);

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
    
    
    
    ce_cam_i2c_write(camlr,0x70,0x0000);
    
    ce_cam_set_af_mode(camlr);
    
    ce_cam_i2c_write(camlr,0x0F,0x0000);
    ce_cam_i2c_write(camlr,0xAC,0x0001);
    ce_cam_i2c_write(camlr,0xAD,0x01E0);
    ce_cam_i2c_write(camlr,0xAB,0x0040);
    ce_cam_i2c_write(camlr,0xB0,0xFFFF);    
    ce_cam_i2c_write(camlr,0xA5,0x0037);
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
    
    /*recommended register setting and performance impact   PDF-page14*/
    ce_cam_i2c_write(camlr,0x20,0x03C7);
    ce_cam_i2c_write(camlr,0x24,0x001B);
    ce_cam_i2c_write(camlr,0x2B,0x0003);
    ce_cam_i2c_write(camlr,0x2F,0x0003);
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
    
    ce_cam_set_mt9v034_config_default(camlr);

    ce_cam_set_mt9v034_fps(camlr);
    
    ce_cam_set_mt9v034_EG_mode(camlr);

    ce_cam_ctrl_camera(camlr,SET_MCLK_12MHz);

    usleep(1000);

    ce_cam_sync_rst(camlr);

    struct timeval cap_systime;

    int r,transferred = 0;
    unsigned char pass;

    img_pkg *timg_pkg;
    threadsafe_queue<img_pkg *> *tlist;

    if(CAMD1_LEFT == camlr)
    {
        tlist = &img_pkg_left_list;
    }
    else if(CAMD1_RIGHT == camlr)
    {
        tlist = &img_pkg_right_list;
    }


    libusb_device_handle *pcam_handle;
    pcam_handle = ce_cam_get_cam_handle(camlr);

    while(!ce_cam_capture_stop_run)
    {
        timg_pkg = new img_pkg;
        if (NULL == timg_pkg)
        {
            LOG("ce_cam_capture alloc memory failure! exit thread!\r\n");
            break;
        }

        memset(timg_pkg, 0, sizeof(img_pkg));
        r = libusb_bulk_transfer(pcam_handle, 0x82, timg_pkg->data, ce_config_get_cf_img_buff_size(), &transferred, 1000);
        gettimeofday(&cap_systime,NULL);
        timg_pkg->timestamp = cap_systime.tv_sec+0.000001*cap_systime.tv_usec-ce_config_get_cf_img_time_offset();

        if(r)
        {
            LOG("cam %d bulk transfer returned: %d\n",camlr,r);
        }

        pass = (timg_pkg->data[ce_config_get_cf_img_size()+0]==0xFF
                &&timg_pkg->data[ce_config_get_cf_img_size()+1]==0x00
                &&timg_pkg->data[ce_config_get_cf_img_size()+2]==0xFE
                &&timg_pkg->data[ce_config_get_cf_img_size()+3]==0x01);

        if((pass == 1)&&(ce_cam_rst_flag_left == false)&&(ce_cam_rst_flag_right == false))
        {

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
            LOG("cam %d bulk transfer check failed: %d\n",camlr,r);
            
            ce_cam_ctrl_camera(camlr,SET_MCLK_12MHz);
        }
    }

    pthread_exit(NULL);
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






int ce_cam_capture_init()
{
    int ret_val=0;

    int r;
    r = ce_usb_open();
    if(r<1)
    {
        LOG("celog: No cameras found! r = %d\r\n",r);
        return ERROR;
    }
    else
    {
        LOG("celog: Number of device of interest found: %d\r\n",r);
    }

    libusb_device_handle *pusb_handle;
    for(int i=0; i<r; i++)
    {
        unsigned char buf = 0;
        pusb_handle = ce_usb_gethandle(i);
        int r_num = libusb_control_transfer(pusb_handle,RT_D2H,GET_CAM_LR,0,0,&buf,1,1000);
        if(r_num != 1)
        {
            LOG("celog: Get the device LR addr failed");
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
        LOG("celog: lost a cam left or right \r\n");
        return ERROR;
    }
    else
    {
        int temp=0;
        int caml_addr = CAMD1_LEFT;
        int camr_addr = CAMD1_RIGHT;

  
        ce_cam_get_soft_version(caml_addr);
        ce_cam_get_soft_version(camr_addr);

        
        
        if(ce_config_get_cf_cam_mode()& CAMD1_LEFT_ENABLE)
        {
            temp = pthread_create(&ce_camd1l_capture_thread, NULL, ce_cam_capture, &caml_addr);
            if(temp)
            {
                LOG("celog: create thread failed !\r\n");
                return ERROR;
            }
        }

        if(ce_config_get_cf_cam_mode() & CAMD1_RIGHT_ENABLE)
        {
            temp = pthread_create(&ce_camd1r_capture_thread, NULL, ce_cam_capture, &camr_addr);
            if(temp)
            {
                LOG("celog: create thread failed !\r\n");
                return ERROR;
            }
        }

        usleep(1000);
        return SUCCESS;
    }
}

void ce_cam_capture_close()
{
    ce_cam_capture_stop_run=true;


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
        cv::imshow("left",img_left);
        std::cout << "left tamps:" << std::setprecision(15) << img_lr_pkg->left_img->timestamp << std::endl;

        memcpy(img_right.data,img_lr_pkg->right_img->data,ce_config_get_cf_img_size());
        cv::imshow("right",img_right);
        std::cout << "right tamps:" << std::setprecision(15) << img_lr_pkg->right_img->timestamp << std::endl;
        cv::waitKey(1);

        delete img_lr_pkg->left_img;
        delete img_lr_pkg->right_img;
        delete img_lr_pkg;
    }
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
    if(ce_cam_showimg_thread != 0)
    {
        pthread_join(ce_cam_showimg_thread,NULL);
    }

}

static void* ce_cam_preprocess(void *)
{
    cv::Mat  img_left(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);

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
