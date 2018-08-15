
#include "yuv422_img.h"

yuv422_img::yuv422_img(int width, int height)
{
    this->width  = width;
    this->height = height;
}

cv::Mat* yuv422_img::grab_uyvy_frame(char* frame_raw)
{
    cv::Mat mat_frame_raw(1, (this->width * 2) * this->height, CV_8UC1, frame_raw);
    cv::Mat channels[4];

    mat_frame_raw = mat_frame_raw.reshape(4);
    cv::split(mat_frame_raw, channels);

// YUYV FORMAT
///*
    cv::Mat& U  = channels[1];
    cv::Mat& Y1 = channels[0];
    cv::Mat& V  = channels[3];
    cv::Mat& Y2 = channels[2];
//*/
// UYVY FORMAT
/*
    cv::Mat& U  = channels[0];
    cv::Mat& Y1 = channels[1];
    cv::Mat& V  = channels[2];
    cv::Mat& Y2 = channels[3];
*/


    cv::Mat UV_arr[2] = { U, V };
    cv::Mat YY_arr[2] = { Y1, Y2 };
    cv::Mat UV, YY;
    
    cv::merge(UV_arr, 2, UV);
    cv::merge(YY_arr, 2, YY);

    UV = UV.reshape(1, this->height);
    YY = YY.reshape(1, this->height);

    cv::Mat UVYY_arr[2] = {UV, YY};
    cv::Mat* mat_frame_uyvy = new cv::Mat(this->height, this->width, CV_8UC3);

    cv::merge(UVYY_arr, 2, *mat_frame_uyvy);

    return mat_frame_uyvy;
}




