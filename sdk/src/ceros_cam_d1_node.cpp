#include "ceros_cam_d1_driver.h"
int g_nCtrl = 1;

int main(int argc, char**argv)
{
    ros::init(argc, argv, "cooleye_d1");
    ros::NodeHandle nh;
    char *path = "cecfg_std.txt";
    INIT_LOG(1000, 1000, LOGMSG_LEVEL_DEBUG);

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

    if(argv[1])
    {
        path = argv[1];
    }

    CoolEyeCamD1 CoolEyeCamD1RUN(nh, path);

    if(!CoolEyeCamD1RUN.start_cam_D1_preprocess())
        std::cout << "ERROR: camera preprocess open failed" << std::endl;

    if(!CoolEyeCamD1RUN.start_cam_D1_capture())
        std::cout << "ERROR: camera open failed" << std::endl;

    if(!CoolEyeCamD1RUN.start_imu_icm26089())
        std::cout << "ERROR: imu open failed" << std::endl;

    CoolEyeCamD1RUN.exectu();

    FINI_LOG();

    return 0;
}
