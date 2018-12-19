
#include "ceros_cam_d1_driver.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "cooleye_d1");
    ros::NodeHandle nh;
    char *path = "cecfg_std.txt";

    if(argv[1])
    {
        path = argv[1];
    }

    CoolEyeCamD1 CoolEyeCamD1RUN(nh, path);

    if(!CoolEyeCamD1RUN.start_cam_D1_capture())
        std::cout << "ERROR: camera open failed" << std::endl;
    if(!CoolEyeCamD1RUN.start_cam_D1_preprocess())
        std::cout << "ERROR: camera preprocess open failed" << std::endl;
    if(!CoolEyeCamD1RUN.start_imu_icm26089())
        std::cout << "ERROR: imu open failed" << std::endl;

    CoolEyeCamD1RUN.exectu();

    return 0;
}
