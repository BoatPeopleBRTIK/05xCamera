#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <string>

#include "yuv422_img.h"

int main(int argc, char** argv){
	
	yuv422_img imgCapture(WIDTH, HEIGHT);
	


#ifdef TEST	
	 char* frame_raw = new char[(WIDTH * 2) * HEIGHT];//qcif format
	 std::cin.read(frame_raw, (WIDTH * 2) * HEIGHT);
#endif

#ifndef TEST
	std::ifstream yuv_file("/home/shivam/paho.mqtt.c/build/output/test/mqtt_recd_frame.yuv");
	int i;
	char temp;
	char* frame_raw = new char[(WIDTH * 2) * HEIGHT];
	
	for(i=0;i < (WIDTH*HEIGHT/2);i++){
		yuv_file.get(temp); 
		frame_raw[4*i] = temp;
		yuv_file.get(temp); 
		frame_raw[4*i + 3] = temp;
		yuv_file.get(temp); 
		frame_raw[4*i + 2] = temp;
		yuv_file.get(temp); 
		frame_raw[4*i + 1] = temp;
	}	
	yuv_file.close();
#endif

	cv::Mat* mat_frame_uyvy = imgCapture.grab_uyvy_frame(frame_raw);

    	cv::Mat mat_frame_bgr;
    	cv::cvtColor(*mat_frame_uyvy, mat_frame_bgr, cv::COLOR_YUV2BGR_UYVY);
			
	cv::imshow("", mat_frame_bgr);
    	cv::waitKey(0);
	
	delete mat_frame_uyvy;
	delete[] frame_raw;

}




