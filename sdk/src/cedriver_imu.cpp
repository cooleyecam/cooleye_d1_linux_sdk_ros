#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <sys/time.h>
#include <linux/types.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>

#include "cedriver_imu.h"
#include "cedriver_global_config.h"
#include "threadsafe_queue.h"
#include "cedriver_config.h"
#include "celib_MadgwickAHRS.h"


threadsafe_queue<icm20689_pkg *> icm20689_pkg_list;

pthread_t ce_imu_capture_thread;
bool ce_imu_capture_stop_run = false;

pthread_t ce_imu_showdata_thread;
bool ce_imu_showdata_stop_run = false;


int ce_imu_uart_fd = 0;

static int ce_imu_find_pkg_head(unsigned char* buf,int len)
{
    int i;
    for(i=0; i<len-1; i++)
    {
        if(buf[i] == CMD_HEAD1)
            if(buf[i+1] == CMD_HEAD2)
                return i;
    }
    if(buf[len-1] == CMD_HEAD1)
        return -1;
    return -2;
}

static float ce_imu_data_norm(float * q, int length)
{
    float datasum = 0.0;
    for(int i=0; i<length; i++) datasum += q[i] * q[i];
    return sqrt(datasum);
}

static int ce_imu_q_normalize(float* q)
{
    float qnorm = ce_imu_data_norm(q, 4);
    for(int i=0; i<4; i++) q[i] /= qnorm;
}

static int ce_imu_open_uart(const char* dev_str)
{
    int fd = open(dev_str, O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd)
    {
        perror("celog: uart open error !");
        return(-1);
    }

    if(fcntl(fd, F_SETFL, 0) < 0)
        printf("celog: fcntl failed!\n");
    if(isatty(STDIN_FILENO) == 0)
        printf("celog: standard input is not a terminal device\n");
    return fd;
}

static int ce_imu_set_uart(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if ( tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;    
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 16;
    tcflush( fd, TCIFLUSH);
    if((tcsetattr( fd, TCSANOW, &newtio))!=0)
    {
        perror("uart set error");
        return -1;
    }
    return 0;
}

static int ce_imu_format_imu_frame(unsigned char* imu_frame, double timestamp, icm20689_pkg *ticm20689_pkg)
{

    float RotMat[3][3];
    ce_config_get_cf_imu_icm20689_acc_T((float **)RotMat);
    
    float acc_biasX = ce_config_get_cf_imu_icm20689_acc_bias_X();
    float acc_biasY = ce_config_get_cf_imu_icm20689_acc_bias_Y();
    float acc_biasZ = ce_config_get_cf_imu_icm20689_acc_bias_Z();
    
    float acc_offset[3] = {acc_biasX,  acc_biasY, acc_biasZ};

    float gyro_biasX = ce_config_get_cf_imu_icm20689_gyro_bias_X();
    float gyro_biasY = ce_config_get_cf_imu_icm20689_gyro_bias_Y();
    float gyro_biasZ = ce_config_get_cf_imu_icm20689_gyro_bias_Z();
    
    float gyro_offset[3] = {gyro_biasX,  gyro_biasY, gyro_biasZ};
    
    
    
    
    int imu_data[6];

    imu_data[0] = *(short*)(&imu_frame[3]);
    imu_data[1] = *(short*)(&imu_frame[5]);
    imu_data[2] = *(short*)(&imu_frame[7]);
    
    imu_data[3] = *(short*)(&imu_frame[9]);
    imu_data[4] = *(short*)(&imu_frame[11]);
    imu_data[5] = *(short*)(&imu_frame[13]);
    
    
    ticm20689_pkg->num = imu_frame[2];

    float fgyro_nobias[3]= {
        ((float)imu_data[0]-gyro_offset[0]),
        ((float)imu_data[1]-gyro_offset[1]),
        ((float)imu_data[2]-gyro_offset[2])};
    
    ticm20689_pkg->rx = 1.0*fgyro_nobias[0]/32768*500;
    ticm20689_pkg->ry = 1.0*fgyro_nobias[1]/32768*500;
    ticm20689_pkg->rz = 1.0*fgyro_nobias[2]/32768*500;
    
    
    
    float facc_nobias[3]= {
        ((float)imu_data[3]-acc_offset[0]),
        ((float)imu_data[4]-acc_offset[1]),
        ((float)imu_data[5]-acc_offset[2])};
    
    
        
    float facc_t[3] ={0,0,0};

    facc_t[0] = 1.0*facc_nobias[0]/8192*STD_g;
    facc_t[1] = 1.0*facc_nobias[1]/8192*STD_g;
    facc_t[2] = 1.0*facc_nobias[2]/8192*STD_g;    
    
//     ticm20689_pkg->ax = 1.0*facc_nobias[0]/8192*STD_g;
//     ticm20689_pkg->ay = 1.0*facc_nobias[1]/8192*STD_g;
//     ticm20689_pkg->az = 1.0*facc_nobias[2]/8192*STD_g;
    
    ticm20689_pkg->ax = RotMat[0][0]*facc_t[0] + RotMat[0][1]*facc_t[1] + RotMat[0][2]*facc_t[2];
    ticm20689_pkg->ay = RotMat[1][0]*facc_t[0] + RotMat[1][1]*facc_t[1] + RotMat[1][2]*facc_t[2];
    ticm20689_pkg->az = RotMat[2][0]*facc_t[0] + RotMat[2][1]*facc_t[1] + RotMat[2][2]*facc_t[2];
    

    ticm20689_pkg->timestamp = timestamp - IMU_DELAY_OFFSET;

}



static int ce_imu_cmd_icm20689_start()
{
    
    unsigned char tcmd[10];

    tcmd[0] = CMD_HEAD1;
    tcmd[1] = CMD_HEAD2;
    tcmd[2] = CMD_IMC20689_START;

    short int acc_offset[3] ;
                                
    acc_offset[0] = (short int)ce_config_get_cf_imu_icm20689_acc_bias_X();
    acc_offset[1] = (short int)ce_config_get_cf_imu_icm20689_acc_bias_Y();
    acc_offset[2] = (short int)ce_config_get_cf_imu_icm20689_acc_bias_Z();                                
                                
    memcpy(&tcmd[3], acc_offset, 6);

    tcmd[9] = 0;
    for(int i=3; i<9; i++)
    {
        tcmd[9] += tcmd[i];
    }

    int ret = write(ce_imu_uart_fd, tcmd, 10);
    usleep(10000);
    return ret;
}



static int ce_imu_cmd_sample_rate()
{
    
    unsigned char tcmd[10] = {0,0,0,0,0, 0,0,0,0,0};

    tcmd[0] = CMD_HEAD1;
    tcmd[1] = CMD_HEAD2;
    tcmd[2] = CMD_IMC20689_SAMPLE_RATE;

    tcmd[3] = 1000/ce_config_get_cf_imu_icm20689_sample_rate() - 1 ;
    
    tcmd[9] = 0;
    for(int i=3; i<9; i++)
    {
        tcmd[9] += tcmd[i];
    }
    
    int ret = write(ce_imu_uart_fd, tcmd, 10);
    usleep(10000);
    return ret;
}



void *ce_imu_data_feed(void*)
{
    unsigned char imu_frame[IMU_FRAME_LEN];
    unsigned char imu_frame_buf[2*IMU_FRAME_LEN];

    memset(imu_frame, 0, IMU_FRAME_LEN);
    memset(imu_frame_buf, 0, 2*IMU_FRAME_LEN);

    double time_interval = (double)1/ce_config_get_cf_imu_icm20689_sample_rate();
    double timestamp;
    double start_time;
    double last_time;
    double new_time;
    double last_pred;
    double new_pred;

    struct timeval getIMUTime;

    //tcflush(ce_imu_uart_fd, TCIFLUSH);
    gettimeofday(&getIMUTime, NULL);


    start_time = getIMUTime.tv_sec + 0.000001 * getIMUTime.tv_usec;

    new_time = start_time;
    new_pred = start_time;

    last_time =  start_time - (double)1/ce_config_get_cf_imu_icm20689_sample_rate();
    last_pred =  start_time - (double)1/ce_config_get_cf_imu_icm20689_sample_rate();
    
    int num_get = 0;
    
    tcflush(ce_imu_uart_fd, TCIFLUSH);
    while(!ce_imu_capture_stop_run)
    {        
        start_time = getIMUTime.tv_sec + 0.000001 * getIMUTime.tv_usec;

        while(num_get < IMU_FRAME_LEN)
        {
            int temp_read_get = read(ce_imu_uart_fd, &imu_frame_buf[num_get], IMU_FRAME_LEN);
            if (0 >= temp_read_get)
            {
                break;
            }
            num_get += temp_read_get;
        }
        
        if (0 == num_get)
            continue;
        
        //std::cout << "start_time :"<< std::setprecision(15)  << start_time <<std::endl;
        gettimeofday(&getIMUTime, NULL);

        int position_pkghead = ce_imu_find_pkg_head(imu_frame_buf, num_get);
        if(position_pkghead == -2)
        {
            memset(imu_frame_buf, 0, 2*IMU_FRAME_LEN);
            num_get = 0;
            continue;
        }
        else if(position_pkghead == -1)
        {
            memset(imu_frame_buf, 0, 2*IMU_FRAME_LEN);
            num_get = 1;
            imu_frame_buf[0] = CMD_HEAD1;
            continue;
        }

        int valid_len = num_get - position_pkghead;
        
        if(valid_len<IMU_FRAME_LEN)
        {
            for(int i=0; i<valid_len; i++)
                imu_frame_buf[i] = imu_frame_buf[position_pkghead+i];

            for(int i=valid_len; i<2*IMU_FRAME_LEN; i++)
                imu_frame_buf[i] = 0;

            num_get = valid_len;
            continue;
        }
        
        unsigned char checksum = 0;
        for(int i=2; i<(IMU_FRAME_LEN-1); i++)checksum += imu_frame_buf[position_pkghead+i];
        if(checksum != imu_frame_buf[position_pkghead+(IMU_FRAME_LEN-1)])
        {
            for(int i=2; i<valid_len; i++)
                imu_frame_buf[i-2] = imu_frame_buf[position_pkghead+i];

            for(int i=valid_len-2; i<2*IMU_FRAME_LEN; i++)
                imu_frame_buf[i] = 0;

            num_get = valid_len-2;
            continue;
        }
        
        if(valid_len > IMU_FRAME_LEN)
        {
            tcflush(ce_imu_uart_fd, TCIFLUSH);
            memset(imu_frame_buf, 0, 2*IMU_FRAME_LEN);
            num_get = 0;
            continue;
        }

        new_time = getIMUTime.tv_sec + 0.000001*getIMUTime.tv_usec;

        double newtime_interval = new_time-last_time;
        if(newtime_interval<0.004 || newtime_interval>0.006)
            newtime_interval = 0.005;
        time_interval = time_interval*0.999 + 0.001*newtime_interval;

        if(new_time-last_pred < time_interval)
            new_pred = new_time;
        else
            new_pred = last_pred + time_interval + 0.001*(new_time - last_pred - time_interval);

        last_time = new_time;
        last_pred = new_pred;

        memcpy(imu_frame, &imu_frame_buf[position_pkghead], IMU_FRAME_LEN);
        timestamp = new_pred;

        icm20689_pkg *ticm20689_pkg = new icm20689_pkg;
        if (NULL == ticm20689_pkg)
        {
            printf("ce_imu_data_feed alloc memory failure! exit thread!\n");
            break;
        }
        memset(ticm20689_pkg, 0, sizeof(ticm20689_pkg));

        ce_imu_format_imu_frame(imu_frame, timestamp, ticm20689_pkg);


        icm20689_pkg *icm20689_giveup = NULL;
        icm20689_pkg_list.push(ticm20689_pkg, icm20689_giveup, IMU_FIFO_SIZE - 5);
        if (NULL != icm20689_giveup)
        {
            delete icm20689_giveup;
            icm20689_giveup = NULL;
        }    
        
        memset(imu_frame_buf, 0, 2*IMU_FRAME_LEN);
        num_get = 0;

    }

    ce_imu_capture_thread = 0;
    pthread_exit(NULL);
}

int ce_imu_capture_init()
{
    int fd = ce_imu_open_uart(ce_config_get_cf_imu_uart().c_str());
    if(fd < 0)
    {
        printf("celog: open_uart failed !\r\n");
        return ERROR;
    }

    if(ce_imu_set_uart(fd, 460800, 8, 'N', 1) < 0)
    {
        printf("celog: set_uart failed !\r\n");
        return ERROR;
    }

    ce_imu_uart_fd = fd;

    if(ERROR == ce_imu_cmd_icm20689_start())
    {
        printf("celog: icm20689_start failed !\r\n");
        return ERROR;
    }
    
    
    
    if(ERROR == ce_imu_cmd_sample_rate())
    {
        printf("celog: icm20689 set sample rate failed !\r\n");
        return ERROR;
    }


    int temp;
    temp = pthread_create(&ce_imu_capture_thread, NULL, ce_imu_data_feed, NULL);
    if( temp )
    {
        printf("celog: Thread imu creat fail !\r\n");
        return ERROR;
    }

    return SUCCESS;
}

void ce_imu_capture_close()
{
    ce_imu_capture_stop_run = true;
    usleep(10000);
    
    if(ce_imu_capture_thread !=0)
    {
        pthread_join(ce_imu_capture_thread, NULL);
    }
}

static void* ce_imu_showdata(void *)
{
    icm20689_pkg *ticm20689_pkg;
    while(!ce_imu_showdata_stop_run)
    {
        if(!icm20689_pkg_list.try_pop(ticm20689_pkg))
        {
            usleep(100);
            continue;
        }


         std::cout << "imu_stamp :" << std::setprecision(15)<< ticm20689_pkg->timestamp << std::endl;
//         std::cout << "Acc X :" << ticm20689_pkg->ax << std::endl;
//         std::cout << "Acc Y :" << ticm20689_pkg->ay << std::endl;
//         std::cout << "Acc Z :" << ticm20689_pkg->az << std::endl;
// 
//         std::cout << "Gyr X :" << ticm20689_pkg->rx << std::endl;
//         std::cout << "Gyr Y :" << ticm20689_pkg->ry << std::endl;
//         std::cout << "Gyr Z :" << ticm20689_pkg->rz << std::endl;

//       MadgwickAHRSupdateIMU( 3.1415926f * ticm20689_pkg->rx / 180.0f,
//                             3.1415926f * ticm20689_pkg->ry / 180.0f,
//                             3.1415926f * ticm20689_pkg->rz / 180.0f,
//                             ticm20689_pkg->ax,
//                             ticm20689_pkg->ay,
//                             ticm20689_pkg->az);

        delete ticm20689_pkg;
        ticm20689_pkg = NULL;


    }
    ce_imu_showdata_thread = 0;
    pthread_exit(NULL);
}

int ce_imu_showdata_init()
{
    int temp = pthread_create(&ce_imu_showdata_thread, NULL, ce_imu_showdata, NULL);
    if(temp)
    {
        printf("Failed to create thread show_imuData\r\n");
        return ERROR;
    }
    return SUCCESS;
}

void ce_imu_showdata_close()
{    
    ce_imu_showdata_stop_run = true;
    usleep(100);
    
    if(ce_imu_showdata_thread != 0)
    {
        pthread_join(ce_imu_showdata_thread,NULL);
    }    
}


void ce_imu_clear_list()
{
    
    icm20689_pkg *ticm20689_pkg;
    int tsize = 0;

    while(icm20689_pkg_list.size() != 0)
    {
        if(!icm20689_pkg_list.try_pop(ticm20689_pkg))
        {
            continue;
        }
        
        delete ticm20689_pkg;
        ticm20689_pkg = NULL;
        tsize = icm20689_pkg_list.size();
        
    }
    
}


void ce_imu_get_acc_data(short int *ptr)
{
    int cnt = 0;
    short int *ptr_raw;
    ptr_raw = ptr;
    icm20689_pkg *ticm20689_pkg;
 
    ce_imu_clear_list();
 
    while(cnt<=1024)
    {
        
        if(!icm20689_pkg_list.try_pop(ticm20689_pkg))
        {
            usleep(10);
            continue;
        }

        
        float norm_gyro = sqrt( ticm20689_pkg->rx * ticm20689_pkg->rx +
                                ticm20689_pkg->ry * ticm20689_pkg->ry +
                                ticm20689_pkg->rz * ticm20689_pkg->rz);
        
        if(norm_gyro > 10)
        {
            std::cout << "celog: don't move the camera..." << std::endl;
            cnt = 0;
            ptr = ptr_raw;
            sleep(1);
            continue;
        }
        
        
        
        *ptr = (short int)(ticm20689_pkg->ax/STD_g*8192);
        ptr++;
        *ptr = (short int)(ticm20689_pkg->ay/STD_g*8192);
        ptr++;
        *ptr = (short int)(ticm20689_pkg->az/STD_g*8192);
        ptr++;
        
        delete ticm20689_pkg;
        ticm20689_pkg = NULL;
        cnt++;
        
        if(cnt%128 == 0)
        {
            std::cout << "cnt :" << cnt <<"/1024" << std::endl;
        }
        
    }
}



void ce_imu_get_gyro_data(short int *ptr)
{
    int cnt = 0;
    short int *ptr_raw;
    ptr_raw = ptr;
    
    icm20689_pkg *ticm20689_pkg;
    
 
    ce_imu_clear_list();
 
    while(cnt<=1024*5)
    {
        
        if(!icm20689_pkg_list.try_pop(ticm20689_pkg))
        {
            usleep(10);
            continue;
        }

        
        float norm_gyro = sqrt( ticm20689_pkg->rx * ticm20689_pkg->rx +
                                ticm20689_pkg->ry * ticm20689_pkg->ry +
                                ticm20689_pkg->rz * ticm20689_pkg->rz);
        
        if(norm_gyro > 10)
        {
            std::cout << "celog: don't move the camera..." << std::endl;
            cnt = 0;
            ptr = ptr_raw;
            sleep(1);
            continue;
        }
        
        *ptr = (short int)(ticm20689_pkg->rx/500*32768);
        ptr++;
        *ptr = (short int)(ticm20689_pkg->ry/500*32768);
        ptr++;
        *ptr = (short int)(ticm20689_pkg->rz/500*32768);
        ptr++;
        
        delete ticm20689_pkg;
        ticm20689_pkg = NULL;
        cnt++;
        
        if(cnt%512 == 0)
        {
            std::cout << "cnt :" << cnt <<"/5120" << std::endl;
        }
        
        
    }
}
