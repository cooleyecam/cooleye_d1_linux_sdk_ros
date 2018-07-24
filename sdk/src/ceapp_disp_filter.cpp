#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <iomanip>

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

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

pthread_t ce_disp_filter_thread;
bool ce_disp_filter_stop_run = false;

bool g_bSaveImg = false;
int g_nCtrl = 1;

extern threadsafe_queue<d1_img_output_pkg *> img_pkg_list_d1;



Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

int disp_filter(Mat& mat_Left, Mat& mat_Right, Mat& mat_raw_disp_vis, Mat& mat_filtered_disp_vis)
{
    // parameters-init;
    bool no_downscale = true;
    int max_disp = 128;
    double lambda = 8000.0;
    double sigma  = 1.5 ;
    double vis_mult = 1.0;
    int wsize = -1;

    String algo = "sgbm";
    String filter = "wls_conf";
    

    if(wsize<0) //user provided window_size value
    {
        if(algo=="sgbm")
            wsize = 3; //default window size for SGBM
        else if(!no_downscale && algo=="bm" && filter=="wls_conf")
            wsize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
        else
            wsize = 15; //default window size for BM on full-sized views
    }

    
    //! [load_views]
    Mat left  = mat_Left;
    Mat right = mat_Right;

    //! [load_views]

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;
    
    if(max_disp<=0 || max_disp%16!=0)
    {
        cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
        return -1;
    }
    if(wsize<=0 || wsize%2!=1)
    {
        cout<<"Incorrect window_size value: it should be positive and odd";
        return -1;
    }
    
    if(filter=="wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
    {
        
        if(!no_downscale)
        {
             
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            //! [downscale]
            max_disp/=2;
            if(max_disp%16!=0)
                max_disp += 16-(max_disp%16);
            resize(left ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
            resize(right,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
            //! [downscale]
        }
        else
        {
            
            left_for_matcher  = left.clone();
            right_for_matcher = right.clone();
        }


        if(algo=="bm")
        {
            //! [matching]
            Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            //! [matching]
            
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
            left_matcher->setP1(24*wsize*wsize);
            left_matcher->setP2(96*wsize*wsize);
            left_matcher->setPreFilterCap(63);
            left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            cout<<"Unsupported algorithm";
            return -1;
        }

        //! [filtering]
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        filtering_time = (double)getTickCount();
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);
        filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
        //! [filtering]
        conf_map = wls_filter->getConfidenceMap();

        // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();
        if(!no_downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            resize(left_disp,left_disp,Size(),2.0,2.0,INTER_LINEAR_EXACT);
            left_disp = left_disp*2.0;
            ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }
    }
    else if(filter=="wls_no_conf")
    {
        /* There is no convenience function for the case of filtering with no confidence, so we
        will need to set the ROI and matcher parameters manually */

        left_for_matcher  = left.clone();
        right_for_matcher = right.clone();

        if(algo=="bm")
        {
            Ptr<StereoBM> matcher  = StereoBM::create(max_disp,wsize);
            matcher->setTextureThreshold(0);
            matcher->setUniquenessRatio(0);

            ROI = computeROI(left_for_matcher.size(),matcher);
            wls_filter = createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33*wsize));

            matching_time = (double)getTickCount();
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> matcher  = StereoSGBM::create(0,max_disp,wsize);
            matcher->setUniquenessRatio(0);
            matcher->setDisp12MaxDiff(1000000);
            matcher->setSpeckleWindowSize(0);
            matcher->setP1(24*wsize*wsize);
            matcher->setP2(96*wsize*wsize);
            matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            ROI = computeROI(left_for_matcher.size(),matcher);
            wls_filter = createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));

            matching_time = (double)getTickCount();
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            cout<<"Unsupported algorithm";
            return -1;
        }

        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        filtering_time = (double)getTickCount();
        wls_filter->filter(left_disp,left,filtered_disp,Mat(),ROI);
        filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    }
    else
    {
        cout<<"Unsupported filter";
        return -1;
    }

    
    //collect and print all the stats:
    cout.precision(2);
    cout<<"Matching time:  "<<matching_time<<"s"<<endl;
    cout<<"Filtering time: "<<filtering_time<<"s"<<endl;
    cout<<endl;


    getDisparityVis(left_disp,mat_raw_disp_vis,vis_mult);
    getDisparityVis(filtered_disp,mat_filtered_disp_vis,vis_mult);

    //! [visualization]

    return 0;
}


static void* ce_disp_filter(void *)
{
    ce_config_get_cf_cam_rectify_force_on();
    
    cv::Mat img_left(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);
    cv::Mat img_right(cv::Size(ce_config_get_cf_img_width(),ce_config_get_cf_img_height()),CV_8UC1);

    cv::Mat disp8_raw,disp8_filter;
    cv::Mat disRGB_raw,disRGB_filter;;
    cv::Mat result_raw;
    cv::Mat result_filter;
    
    ////////////////////////////////////////////////////////////////////////////////////

    d1_img_output_pkg *img_lr_pkg;

    while(!ce_disp_filter_stop_run)
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
        
    
        disp_filter(img_left, 
                    img_right, 
                    disp8_raw,
                    disp8_filter);
     
        ce_depth2color(disRGB_raw, disp8_raw, 255, 0);
        ce_depth2color(disRGB_filter, disp8_filter, 255, 0);
        
        
        ce_merge_img(result_raw,disp8_raw,disp8_filter);
        ce_merge_img(result_filter,disRGB_raw,disRGB_filter);
        
        cv::imshow("result_raw",result_raw);
        cv::imshow("result_filter",result_filter);
        
        //reprojectImageTo3D(disp, xyz, Q, true);   
        //imshow("xyz", xyz);
        
        cv::waitKey(1);

        delete img_lr_pkg->left_img;
        delete img_lr_pkg->right_img;
        delete img_lr_pkg;
    }
    ce_disp_filter_thread = 0;
    pthread_exit(NULL);
}


int ce_disp_filter_init()
{
    int temp = pthread_create(&ce_disp_filter_thread, NULL, ce_disp_filter, NULL);
    if(temp)
    {
        printf("celog: Failed to create thread show image \r\n");
        return ERROR;
    }
    return SUCCESS;
}

void ce_disp_filter_close()
{
    ce_disp_filter_stop_run = true;
    if(ce_disp_filter_thread != 0)
    {
        pthread_join(ce_disp_filter_thread,NULL);
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

            r = ce_disp_filter_init();
            if(r < 0)
                printf("celog: disp_filter image error \r\n");
            else
                printf("celog: disp_filter image sucess \r\n");
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
    ce_disp_filter_close();
    return 0;
}












