#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera.h"

//  Real time Capture Video
void CameraParameterSet(std::vector <cv::VideoCapture>& Camerahande){
	for(int i = 0; i < CAMERA_NUM; i++)
	{
		Camerahande[i].set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
		Camerahande[i].set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
		Camerahande[i].set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	}
}

void CameraOpen(std::vector<cv::VideoCapture>& cam, std::vector<bool>& res)
{
	for(int i = 0; i < cam.size(); i++)
	{
	 	// Video strem fiel name
	 	std::string f = "video";
	 	f += std::to_string((i+1));
	 	f += "_41.avi";
	 	//std::cout << f << std::endl;
	 	res[i] = cam[i].open(f);
	}
}

bool CameraCheck(std::vector<bool>& res)
{
	bool ready = false;
	for(int i = 0; i < res.size(); i++) 
	{
	 	if(i == 0)
	 	 	ready = res[i];
	 	else
	 	 	ready && res[i];
	}
	return ready;
}
