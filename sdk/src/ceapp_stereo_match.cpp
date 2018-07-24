#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <signal.h>

#include <iomanip>


using namespace std;


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

#include "cedriver_usb.h"
#include "cedriver_cam.h"
#include "cedriver_imu.h"
#include "cedriver_config.h"
#include "cedriver_global_config.h"
#include "celib_img_process.h"



using namespace cv;

pthread_t ce_ste_match_thread;
bool ce_ste_match_stop_run = false;


bool g_bSaveImg = false;

extern threadsafe_queue<d1_img_output_pkg *> img_pkg_list_d1;

int g_nCtrl = 1;



static void* ce_ste_match(void *)
{
    ce_config_get_cf_cam_rectify_force_on();
    
    cv::Mat img_left(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);

    cv::Mat disparity(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat result_rect;
    
    Mat Q;
    Mat xyz;
    Mat disp, disp8;
    Rect roi1, roi2;

    Mat disRGB;
    
    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    
    ////////////////////////////////////////////////////////////////////////////////////
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    float scale;


    alg = ce_config_get_cf_ste_algorithm();

    
    numberOfDisparities = 128;    
    SADWindowSize =15;
    scale = 1.0;
    no_display = false;

    int color_mode = alg == STEREO_BM ? 0 : -1;

    Mat img1 = img_left;
    Mat img2 = img_right;

    Size img_size = img1.size();

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    if(alg==STEREO_HH)
    sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_3WAY)
    sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
      
    ///////////////////////////////////////////////////////////
    
    
    d1_img_output_pkg *img_lr_pkg;

    while(!ce_ste_match_stop_run)
    {
        if(!img_pkg_list_d1.try_pop(img_lr_pkg))
        {
                usleep(1000);
                continue;
        }

        if(img_pkg_list_d1.size() != 0)
        {
                delete img_lr_pkg->left_img;
                delete img_lr_pkg->right_img;
                delete img_lr_pkg;
                continue;
        }
            
        // get the image
        memcpy(img_left.data, img_lr_pkg->left_img->data, ce_config_get_cf_img_size());
        memcpy(img_right.data,img_lr_pkg->right_img->data,ce_config_get_cf_img_size());
        

        img1 = img_left;
        img2 = img_right;
        
        int64 t = getTickCount();
        if( alg == STEREO_BM )
            bm->compute(img1, img2, disp);
        else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
            sgbm->compute(img1, img2, disp);
    
        t = getTickCount() - t;
        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

        //disp = dispp.colRange(numberOfDisparities, img1p.cols);
        if( alg != STEREO_VAR )
            disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        else
            disp.convertTo(disp8, CV_8U);
    
        //reprojectImageTo3D(disp, xyz, Q, true);   
        //imshow("xyz", xyz);
        
        ce_merge_img(result_rect, img_left, img_right);
        
        cv::imshow("result_rect",result_rect);

        imshow("disparity", disp8);
        ce_depth2color(disRGB, disp8, 255, 0);
        imshow("disRGB", disRGB);
    
        cv::waitKey(1);

        delete img_lr_pkg->left_img;
        delete img_lr_pkg->right_img;
        delete img_lr_pkg;
    }
    ce_ste_match_thread = 0;
    pthread_exit(NULL);
}


int ce_ste_match_init()
{
    int temp = pthread_create(&ce_ste_match_thread, NULL, ce_ste_match, NULL);
    if(temp)
    {
        printf("celog: Failed to create thread show image \r\n");
        return ERROR;
    }
    return SUCCESS;
}

void ce_ste_match_close()
{
    ce_ste_match_stop_run = true;
    if(ce_ste_match_thread != 0)
    {
        pthread_join(ce_ste_match_thread,NULL);
    }
}





void SIGINTHandler(int nSig)
{
    printf("capture a SIGINT signal %d\n", nSig);
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

            r = ce_ste_match_init();
            if(r < 0)
                printf("celog: ste_match image error \r\n");
            else
                printf("celog: ste_match image sucess \r\n");
        }
    }

    g_nCtrl = 1;
    
    while(g_nCtrl)
    {
        sleep(100);
    }

    ce_imu_capture_close();
    
    ce_imu_showdata_close();

    ce_cam_capture_close();
    
    ce_ste_match_close();
    return 0;
}


