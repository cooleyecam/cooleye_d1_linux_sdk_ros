#ifndef CELIB_IMG_PROCESS_H
#define CELIB_IMG_PROCESS_H



void ce_merge_img(cv::Mat &dst,cv::Mat &src1,cv::Mat &src2);
void ce_depth2color(cv::Mat & color, const cv::Mat & depth, const double max, const double min);



#endif
