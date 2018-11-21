#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <vector>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "logmsg.h"
#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_imu.h"
#include "cedriver_config.h"

#define LOG printf
#define CE_CAM_SERIAL_FILE_NAME "./serial.log"

int g_nCtrl = 0;
char g_cCamType = CE_CAM_TYPE_D;
extern int g_nForceWriteFlag;

extern int openStdoutFile();
extern int closeStdoutFile();

void SIGINTHandler(int nSig)
{
    if(0 != g_nCtrl)
        g_nCtrl = 0;
    else
        g_nCtrl = 1;
}

int main(int argc, char* argv[])
{
    int r = 0;
    signal(SIGINT, SIGINTHandler);
    
    INIT_LOG(1000, 1000, LOGMSG_LEVEL_DEBUG);
    
    g_nForceWriteFlag = 0;
    g_cCamType = CE_CAM_TYPE_D;
    
    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            switch(argv[i][1])
            {
                case 'f':
                    g_nForceWriteFlag = 1;
                    break;
                case 'S':
                    g_cCamType = CE_CAM_TYPE_S;
                    break;
                case 'D':
                    g_cCamType = CE_CAM_TYPE_D;
                    break;
                default:
                    break;
            }
        }
        
    }        
    
    openStdoutFile();
    //ce_config_load_settings("../config/cecfg_std.txt");
    
    printf("\r\n");
    if (CE_CAM_TYPE_S == g_cCamType)
    {
        if (1 == g_nForceWriteFlag)
            printf("################ \033[1m\033[45;33m单目\033[0m摄像头\033[1m\033[45;33m强制\033[0m烧写模式 ################\r\n\r\n");
        else
            printf("################ \033[1m\033[45;33m单目\033[0m摄像头\033[1m\033[45;33m普通\033[0m烧写模式 ################\r\n\r\n");
        
        r = ce_cam_write_deviceid_S1();
    }
    else
    {
        if (1 == g_nForceWriteFlag)
            printf("################ \033[1m\033[45;33m双目\033[0m摄像头\033[1m\033[45;33m强制\033[0m烧写模式 ################\r\n\r\n");
        else
            printf("################ \033[1m\033[45;33m双目\033[0m摄像头\033[1m\033[45;33m普通\033[0m烧写模式 ################\r\n\r\n");
        
        r = ce_cam_write_deviceid_D1();
    }        
    
    if(r < 0)
    {
        printf("celog: cam capture error \r\n");
    }
    else
    {
        printf("celog: cam capture success \r\n");
    }  
    
    closeStdoutFile();
    
    FINI_LOG();
    return 0;
}
