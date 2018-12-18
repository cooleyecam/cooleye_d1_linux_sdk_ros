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
#include "logmsg.h"
#include "cpu_set.h"

int g_nCtrl = 1;
extern int g_nPara;

void SIGINTHandler(int nSig)
{
    if(0 != g_nCtrl)
    {
        g_nCtrl = 0;
        printf("SIGINTHandler: g_nCtrl = %d\r\n", g_nCtrl);
    }
    else
    {
        g_nCtrl = 1;
        printf("SIGINTHandler: g_nCtrl = %d\r\n", g_nCtrl);
    }
}

int main(int argc, char* argv[])
{
    int r = 0;
    signal(SIGINT, SIGINTHandler);
    INIT_LOG(1000, 1000, LOGMSG_LEVEL_DEBUG);

    if (argc == 2)
    {
        g_nPara = atoi(argv[1]);
    }

    CCpuSet::instance()->m_nCpuPoolSize = 6;
    CCpuSet::instance()->m_aCpuPool[0] = 0;
    CCpuSet::instance()->m_aCpuPool[1] = 1;
    CCpuSet::instance()->m_aCpuPool[2] = 2;
    CCpuSet::instance()->m_aCpuPool[3] = 3;
    CCpuSet::instance()->m_aCpuPool[4] = 1;
    CCpuSet::instance()->m_aCpuPool[5] = 2;
    CCpuSet::instance()->m_nCpuSetFlag = true;

    ce_config_load_settings("../config/cecfg_std.txt");

#if 0
    int fd = ce_imu_capture_init();
    if(fd < 0)
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_imu_capture_init error\r\n");
    }

    else
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_imu_capture_init success\r\n");

        fd = ce_imu_showdata_init();
        if(fd < 0)
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_imu_showdata_init error\r\n");
        else
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_imu_showdata_init success\r\n");
    }
#endif

    r = ce_cam_preprocess_init();
    if(r < 0)
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_preprocess_init error \r\n");
    }
    else
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_cam_preprocess_init  sucess \r\n");

        r = ce_cam_showimg_init();
        if(r < 0)
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_showimg_init  error \r\n");
        }
        else
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_cam_showimg_init  sucess \r\n");
        }
    }

    g_nCtrl = 1;
    r = ce_cam_capture_init();
    if(r < 0)
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_cam_capture_init error \r\n");
    }
    else
    {
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_cam_capture_init success \r\n");
    }

    while(g_nCtrl)
    {
        sleep(1);
    }

#if 0
    ce_imu_capture_close();
    ce_imu_showdata_close();
#endif

    ce_cam_showimg_close();
    usleep(10000);

    FINI_LOG();
    return 0;
}
