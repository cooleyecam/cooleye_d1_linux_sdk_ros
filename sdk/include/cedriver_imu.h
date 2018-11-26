#ifndef CEDRIVER_IMU_H
#define CEDRIVER_IMU_H

typedef struct
{
    unsigned char num;
    float rx,ry,rz;
    float ax,ay,az;
    double timestamp;
}icm20689_pkg;

int ce_imu_capture_init();
void ce_imu_capture_close();

int ce_imu_showdata_init();
void ce_imu_showdata_close();

void ce_imu_get_acc_data(short int *ptr);
void ce_imu_get_gyro_data(short int *ptr);
#endif
