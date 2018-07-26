#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <vector>
#include <sstream>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_imu.h"
#include "cedriver_config.h"

int g_nCtrl;

void SIGINTHandler(int nSig)
{
    if(0 != g_nCtrl)
        g_nCtrl = 0;
    else
        g_nCtrl = 1;
}





int main(int argc, char* argv[])
{
    signal(SIGINT, SIGINTHandler);

    ce_config_load_settings("../config/cecfg_std.txt");

    int fd = ce_imu_capture_init();
    if(fd < 0)
    {
        printf("celog: imu caputre error\r\n");
    }

    else
    {
        printf("celog: imu caputre success\r\n");

        fd = ce_imu_showdata_init();
        if(fd < 0)
            printf("celog: imu show data error\r\n");
        else
            printf("celog: imu show data success\r\n");
    }

    int r = ce_cam_capture_init();
    if(r < 0)
    {
        printf("celog: cam capture error \r\n");
    }
    else
    {
        printf("celog: cam capture success \r\n");

        r = ce_cam_preprocess_init();
        if(r < 0)
        {
            printf("celog: cam preprocess error \r\n");
        }
        else
        {
            printf("celog: cam preprocess sucess \r\n");

            r = ce_cam_showimg_init();
            if(r < 0)
                printf("celog: cam show image error \r\n");
            else
                printf("celog: cam show image sucess \r\n");
        }
    }

    g_nCtrl = 1;
    while(g_nCtrl)
    {
        sleep(1);
    }
    
    ce_imu_capture_close();     
    ce_imu_showdata_close();
    ce_cam_capture_close();
    ce_cam_showimg_close();
    return 0;
}
