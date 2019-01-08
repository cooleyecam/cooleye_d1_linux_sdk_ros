#include <sys/sysinfo.h>
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
extern int g_imgvb;

//#define CE_RAW_DATA_IMU

void SIGINTHandler(int nSig)
{
    g_nCtrl = 0;
    printf("SIGINTHandler: g_nCtrl = %d\r\n", g_nCtrl);
}

int main(int argc, char* argv[])
{
    int r = 0;
    signal(SIGINT, SIGINTHandler);
    INIT_LOG(1000, 1000, LOGMSG_LEVEL_DEBUG);

    if (argc == 2)
    {
        g_imgvb = atoi(argv[1]);
    }

    int nprocs = get_nprocs();
    if (0 == nprocs || 1 == nprocs)
    {
        CCpuSet::instance()->m_nCpuSetFlag = false;
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "The number of cores is %d, and the cores are not bound\r\n", nprocs);
    }
    else
    {
        CCpuSet::instance()->m_nCpuSetFlag = true;
        for (int i = 0; i < nprocs - 1; i++)
        {
            if(i >= CPU_SET_MAX)
                break;

            CCpuSet::instance()->m_aCpuPool[i] = i + 1;
            CCpuSet::instance()->m_nCpuPoolSize++;
        }
        WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO,
            "The number of cores is %d. The binding cores are %d to %d. Core 0 is used for special purposes.\r\n",
            nprocs, 1, CCpuSet::instance()->m_nCpuPoolSize);
    }

    ce_config_load_settings("../config/cecfg_std.txt");

#ifdef CE_RAW_DATA_IMU
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
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "ce_imu_showdata_init error\r\n");
        }
        else
        {
            WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "ce_imu_showdata_init success\r\n");
        }
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

#ifdef CE_RAW_DATA_IMU
    ce_imu_capture_close();
    ce_imu_showdata_close();
#endif

    ce_cam_showimg_close();
    usleep(10000);

    FINI_LOG();
    return 0;
}
