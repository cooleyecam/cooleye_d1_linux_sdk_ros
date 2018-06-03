/**
 * 
 * Implementation of accelerometer calibration.
 *
 * Transform acceleration vector to true orientation, scale and offset
 *
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up
 *                        | -g  0  0 |     // nose down
 *                        |  0  g  0 |     // left side down
 *                        |  0 -g  0 |     // right side down
 *                        |  0  0  g |     // on back
 *                        [  0  0 -g ]     // level
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 * g = 9.80665
 *
 * ===== Rotation =====
 *
 * Calibrating using model:
 * accel_corr = accel_T_r * (rot * accel_raw - accel_offs_r)
 *
 * Actual correction:
 * accel_corr = rot * accel_T * (accel_raw - accel_offs)
 *
 * Known: accel_T_r, accel_offs_r, rot
 * Unknown: accel_T, accel_offs
 *
 * Solution:
 * accel_T_r * (rot * accel_raw - accel_offs_r) = rot * accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * (rot * accel_raw - accel_offs_r) = accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * rot * accel_raw - rot^-1 * accel_T_r * accel_offs_r = accel_T * accel_raw - accel_T * accel_offs)
 * => accel_T = rot^-1 * accel_T_r * rot
 * => accel_offs = rot^-1 * accel_offs_r
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */



#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_imu.h"
#include "cedriver_config.h"
#include <iomanip>
#include <iomanip>


int mat_invert3(float src[3][3], float dst[3][3]);



static void wait_for_enter_key(void)
{
    char c;
    char flag = 0;

    while(flag == 0)
    {
        c = std::cin.get();
        if('\n' == c)
        {
            flag = 1;
        }
    }

}


static void print_the_buff(short int * ptr)
{
    short int x,y,z;
    for(int i=0;i<1024;i++)
    {
        x = *ptr;
        ptr++;
        y = *ptr;
        ptr++;
        z = *ptr;
        ptr++;

        std::cout << x <<" "<< y <<" "<<z<< std::endl;
    }

}



int main(int argc, char* argv[])
{
    short int gyro_buff[1024*5][3];
    float gyro_offs[3] = {0,0,0};
    short int xp_buff[1024][3];
    short int xn_buff[1024][3];
    short int yp_buff[1024][3];
    short int yn_buff[1024][3];
    short int zp_buff[1024][3];
    short int zn_buff[1024][3];

    float accel_ref[6][3];
    float accel_offs[3] = {0,0,0};
    float accel_T_format[3][3];

    memset(gyro_buff,0,sizeof(gyro_buff));
    memset(gyro_offs,0,sizeof(gyro_offs));

    memset(xp_buff,0,sizeof(xp_buff));
    memset(xn_buff,0,sizeof(xn_buff));
    memset(yp_buff,0,sizeof(yp_buff));
    memset(yn_buff,0,sizeof(yn_buff));
    memset(zp_buff,0,sizeof(zp_buff));
    memset(zn_buff,0,sizeof(zn_buff));

    memset(accel_ref,0,sizeof(accel_ref));
    memset(accel_offs,0,sizeof(accel_offs));
    memset(accel_T_format,0,sizeof(accel_T_format));

    /////////////////////////////

    char filepath[] = "../config/cecfg_std.txt";
    std::string config_filepath = filepath;
    ce_config_load_settings(filepath);
    ce_config_rst_imu_offset();

    int fd = ce_imu_capture_init();
    if(fd < 0)
    {
        printf("celog: imu caputre error\r\n");
    }

    else
    {
        printf("celog: imu caputre success\r\n");
    }


    printf("Please press the Enter key and remain static for 5 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_gyro_data(gyro_buff[0]);
    for (unsigned i = 0; i < 1024*5; i++)
    {
        gyro_offs[0] += gyro_buff[i][0];
        gyro_offs[1] += gyro_buff[i][1];
        gyro_offs[2] += gyro_buff[i][2];
    }

    for(int i = 0;i<3;i++)
    {
        gyro_offs[i] = gyro_offs[i] / (1024*5);
    }

    std::cout << "gyro_offs :" << gyro_offs[0] <<" "<< gyro_offs[1] <<" "<< gyro_offs[2] << std::endl;

    printf("Please place the X axis of the camera upside, then press the Enter key and remain static for 1 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_acc_data(xp_buff[0]);


    printf("Please place the X axis of the camera downside, then press the Enter key and remain static for 1 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_acc_data(xn_buff[0]);



    printf("Please place the Y axis of the camera upside, then press the Enter key and remain static for 1 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_acc_data(yp_buff[0]);

    printf("Please place the Y axis of the camera downside, then press the Enter key and remain static for 1 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_acc_data(yn_buff[0]);


    printf("Please place the Z axis of the camera upside, then press the Enter key and remain static for 1 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_acc_data(zp_buff[0]);

    printf("Please place the Z axis of the camera downside, then press the Enter key and remain static for 1 seconds.\r\n");
    wait_for_enter_key();
    ce_imu_get_acc_data(zn_buff[0]);




    for (unsigned i = 0; i < 1024; i++)
    {
		accel_ref[0][0] += xp_buff[i][0];
        accel_ref[0][1] += xp_buff[i][1];
        accel_ref[0][2] += xp_buff[i][2];

        accel_ref[1][0] += xn_buff[i][0];
        accel_ref[1][1] += xn_buff[i][1];
        accel_ref[1][2] += xn_buff[i][2];

		accel_ref[2][0] += yp_buff[i][0];
        accel_ref[2][1] += yp_buff[i][1];
        accel_ref[2][2] += yp_buff[i][2];

        accel_ref[3][0] += yn_buff[i][0];
        accel_ref[3][1] += yn_buff[i][1];
        accel_ref[3][2] += yn_buff[i][2];

        accel_ref[4][0] += zp_buff[i][0];
        accel_ref[4][1] += zp_buff[i][1];
        accel_ref[4][2] += zp_buff[i][2];

        accel_ref[5][0] += zn_buff[i][0];
        accel_ref[5][1] += zn_buff[i][1];
        accel_ref[5][2] += zn_buff[i][2];
    }


    for(int j = 0;j<6;j++)
    {
        for(int i = 0;i<3;i++)
        {
            accel_ref[j][i] = accel_ref[j][i] /1024;
        }
    }


    for (int i = 0; i < 3; i++)
    {
		accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
    }

    
    float accel_offs_format[3];
    for (int i = 0; i < 3; i++)
    {
        accel_offs_format[i] = 1.0*accel_offs[i]/8192*STD_g;
    }
    
    float accel_ref_format[6][3];
    for(int j = 0;j<6;j++)
    {
        for(int i = 0;i<3;i++)
        {
            accel_ref_format[j][i] = 1.0*accel_ref[j][i]/8192*STD_g;
        }
    }
    
    

	float mat_A[3][3];
    memset(mat_A, 0, sizeof(mat_A));
    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned j = 0; j < 3; j++)
        {
            float a = accel_ref_format[i * 2][j] - accel_offs_format[j];
            mat_A[i][j] = a;
        }
    }


	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != SUCCESS)
    {
		while(1)
        {
            printf("celog : error!!! please redo the Calibration again!\r\n");
            sleep(1);
        }
    }

    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned j = 0; j < 3; j++)
        {
            accel_T_format[j][i] = mat_A_inv[j][i]*STD_g;
        }
    }

    
    std::cout << "gyro_offs :" << gyro_offs[0] <<" "<< gyro_offs[1] <<" "<< gyro_offs[2] << std::endl;
    std::cout << "accel_offs :" << accel_offs[0] <<" "<< accel_offs[1] <<" "<< accel_offs[2] << std::endl;
    std::cout << "accel_T_format :" << std::endl;
    std::cout << accel_T_format[0][0] <<" "<< accel_T_format[0][1] <<" "<< accel_T_format[0][2] << std::endl;
    std::cout << accel_T_format[1][0] <<" "<< accel_T_format[1][1] <<" "<< accel_T_format[1][2] << std::endl;
    std::cout << accel_T_format[2][0] <<" "<< accel_T_format[2][1] <<" "<< accel_T_format[2][2] << std::endl;

    std::cout << "filepath :"<< filepath << std::endl;
    ce_config_rewrite_imu_offset(config_filepath, gyro_offs, accel_offs, (float **)accel_T_format);
    
}



int mat_invert3(float src[3][3], float dst[3][3])
{
    float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
                src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
                src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

    if (fabsf(det) < FLT_EPSILON)
    {
        return ERROR;        // Singular matrix
    }

    dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
    dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
    dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
    dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
    dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
    dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
    dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
    dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
    dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

    return SUCCESS;
}
