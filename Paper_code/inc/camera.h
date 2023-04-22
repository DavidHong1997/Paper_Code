#ifndef CAMERASETTING_
#define CAMERASETTING_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "camera.h"

#define CAMERA_NUM 6
#define CAMERA_FPS 15
#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1080

void CameraParameterSet(std::vector <cv::VideoCapture>& );

void CameraOpen(std::vector<cv::VideoCapture>& , std::vector<bool>& );


bool CameraCheck(std::vector<bool>& );
#endif
