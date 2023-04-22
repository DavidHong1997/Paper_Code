#ifndef WARP_
#define WARP_


#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/stitching/warpers.hpp"
#include "time.h"

using namespace std;
using namespace cv;
using namespace cv::detail;


/* Camera 1 Parameter */
Mat K1 = (Mat_<double>(3,3) << 1160.401766089853, 0, 960, 0, 1160.401766089853, 540,0, 0, 1);
Mat R1 = (Mat_<double>(3,3)  << -0.067746162, 0.020170383, 0.99749863,-0.02374129, 0.99948007, -0.02182287,-0.99742019, -0.025160307, -0.067232102);


/* Camera 2 Parametet */
Mat K2 = (Mat_<double>(3,3) << 1150.546463905725, 0, 960,0, 1150.546463905725, 540,0,0,1);
Mat R2 = (Mat_<double>(3,3)  << -0.88771433, -0.012844121, 0.46021536,0.00011054287, 0.99960494, 0.02811113,-0.46039456, 0.025005527, -0.8873623);


/* Camera 3 Parameter */
Mat K3 = (Mat_<double>(3,3) << 1169.942554874235, 0, 960,0, 1169.942554874235, 540,0,0,1);
Mat R3 = (Mat_<double>(3,3)  << -0.68715715, 0.05899857, -0.72410929,0.0096652918, 0.99735147, 0.07208959,0.7264446, 0.042538144, -0.68590736);

/* Camera 4 Parameter */
Mat K4 = (Mat_<double>(3,3) << 1147.378373843317, 0, 960,0, 1147.378373843317, 540,0, 0, 1);
Mat R4 = (Mat_<double>(3,3)  << 0.26879263, 0.01155049, -0.96312881,-0.021102794, 0.99975878, 0.006100364,0.96296698, 0.018685002, 0.26897147); 

/* Camera 5 Parameter */
Mat K5 = (Mat_<double>(3,3) << 1187.60179722193, 0, 960,0, 1187.60179722193, 540,0,0,1);
Mat R5 = (Mat_<double>(3,3)  << 0.92963016, 0.025587482, -0.36760429,-0.0021473479, 0.99794561, 0.064032644,0.36848751, -0.058737315, 0.92777532);

/* Camera 6 Parameter */
Mat K6 = (Mat_<double>(3,3) << 1205.401494669528, 0, 960,0, 1205.401494669528, 540,0,0,1);
Mat R6 = (Mat_<double>(3,3)  << 0.80176187, -0.02481197, 0.59712827,0.015964534, 0.99967057, 0.020102944,-0.59743041, -0.0065848902, 0.80189389);

std::vector<std::vector<cv::Point2d>> projectionPoint(std::vector<cv::Mat>& , 
			   std::vector<cv::Mat>& ,
			   Ptr<RotationWarper> warper,
			   const cv::Point &shift,
			   std::vector<std::vector<cv::Point2d>>& );

std::vector<cv::Point2d> combinePoint(std::vector<std::vector<cv::Point2d>>& );


std::vector<cv::Point2d> predvel(int &,
						   std::vector<cv::Point2d>& ,
						   std::vector<cv::Point2d>& ,
						   std::vector<cv::Point2d>& );


/*
std::vector<cv::Point2f> update(std::vector<cv::Point2f>& , 
								std::vector<cv::Point2f>& , 
								std::vector<cv::Point2f>& );
*/
/*
std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>> update(std::vector<cv::Point2f>& , 
																	  std::vector<cv::Point2f>& , 
																	  std::vector<cv::Point2f>& );
*/
std::vector<cv::Point2d> update(std::vector<cv::Point2d>& , 
											std::vector<cv::Point2d>& , 
											std::vector<cv::Point2d>& );



std::vector<cv::Point2d> sorting(std::vector<cv::Point2d>& , int);



std::vector<double> pixelToBearing(std::vector<cv::Point2d>& );

#endif
