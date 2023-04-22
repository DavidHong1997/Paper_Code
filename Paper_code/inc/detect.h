#ifndef DETECT_
#define DETECT_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "detect.h"

//#define T 1 // waitKey 30 ms

//int T = 10000;

void ImageCapture(std::vector<cv::VideoCapture>& ,std::vector<cv::Mat>& );


bool VisualizeImg(std::vector<cv::Mat>& ,int );


std::vector<cv::Point2d> DetectImg(cv::Mat& img, float, float);


#endif


