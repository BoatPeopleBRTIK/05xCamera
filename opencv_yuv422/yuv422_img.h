#ifndef YUV422_img_H
#define YUV422_img_H

#include <opencv2/core/core.hpp>

//#define TEST 				5   // define test for displaying test qcif image

#ifndef TEST
	#define WIDTH			320
	#define HEIGHT			240
#endif

#ifdef TEST
	#define WIDTH			176
	#define HEIGHT			144
#endif

class yuv422_img
{
public:
    yuv422_img(int width, int height);
    cv::Mat* grab_uyvy_frame(char* frame_raw);

private:
    int width;
    int height;
};

#endif
