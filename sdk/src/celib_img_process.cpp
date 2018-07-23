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


#include "celib_img_process.h"



void ce_merge_img(cv::Mat &dst,cv::Mat &src1,cv::Mat &src2)
{
    CV_Assert(src1.type()==src2.type());
    int rows = src1.rows > src2.rows ? src1.rows : src2.rows;   
    int cols = src1.cols + 20 + src2.cols;
    
    cv::Mat zeroMat = cv::Mat::zeros(rows, cols, src1.type());
    zeroMat.copyTo(dst);
    src1.copyTo(dst(cv::Rect(0,0,src1.cols,src1.rows)));
    src2.copyTo(dst(cv::Rect(src1.cols+20,0,src2.cols,src2.rows)));

//     double score=89.101;
//     char info[256];
//     sprintf(info,"score=%.2f",score);
//     cv::putText(dst,info,cv::Point(2,50),CV_FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,0,0));
}


void ce_depth2color(cv::Mat & color, const cv::Mat & depth, const double max, const double min)
{
    cv::Mat grayImage;
    double alpha = 255.0 / (max - min);
    depth.convertTo(grayImage, CV_8UC1, alpha, -alpha * min  );
    cv::applyColorMap(grayImage, color, cv::COLORMAP_JET);
}



