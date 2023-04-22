#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <queue>

#include "detect.h"
#include "camera.h"
#include "localization.h"
#include "warp.h"
#include "log.h"

#include <fstream>

using std::ofstream;
using std::string;
using std::fstream;

int main()
{
	// Video Input or Camera real time capture
	std::vector<cv::Mat> img(6); // six camera Image frame
	std::vector<cv::VideoCapture> cap(6); // six cap oop create
	std::vector<bool> ret(6);

	std::vector<std::vector<cv::Point2d>> Landmark(6, std::vector<cv::Point2d>(4));	
	std::vector<std::vector<cv::Point2d>> warpMark(6, std::vector<cv::Point2d>(4));
	std::vector<cv::Point2d> resLandmark(4);

	std::vector<cv::Point2d> vel(4); // Four landmark velocity
	std::vector<cv::Point2d> bufLandmark(4);
	std::vector<cv::Point2d> updateMark(4);
	std::vector<cv::Point2d> sortLandmark(4);

	std::vector<cv::Point2d> pt_real(4);
	std::vector<cv::Point2d> pos(4);
	cv::Point2d loc;
	cv::Point2d loc_final;

	std::queue<cv::Point2d> pkg_filt;

	std::vector<double> angle(4);

	int cnt = 0;
	int total = 0;	
	Mat K, R;


	float focal = 1165.17;
	cv::Point shiftCenter(-3660, -608);
	std::vector<cv::Mat> camParam_K(6);
	std::vector<cv::Mat> camParam_R(6);

	// Create Warp to panorama Obj	

	cv::Ptr<cv::WarperCreator> warper_creator = cv::makePtr<cv::CylindricalWarper>();
	cv::Ptr<cv::detail::RotationWarper> warper = warper_creator->create(focal);

	bool ready = false;

	// Open the video file
	CameraOpen(cap, ret);

	// check 6 camera ready
	ready = CameraCheck(ret);
	std::cout << "Camera ready 1/0(true/false):" << ready << std::endl;
	// Load Camear Para H = KRK^-1
	camParam_K[0] = K1;
	camParam_K[1] = K2;
	camParam_K[2] = K3;
	camParam_K[3] = K4;
	camParam_K[4] = K5;
	camParam_K[5] = K6;

	camParam_R[0] = R1;
	camParam_R[1] = R2;
	camParam_R[2] = R3;
	camParam_R[3] = R4;
	camParam_R[4] = R5;
	camParam_R[5] = R6;

	/* Real landmark Landmark A,Landmark B,Landmark C,Landmark D */
	pt_real[0] = pt1;
	pt_real[1] = pt2;
	pt_real[2] = pt3;
	pt_real[3] = pt4;

	ofstream newFile;
    newFile.open("result.txt");
	/***  Main loop ***/
	while(ready){
	 	// Input Image frame
	 	ImageCapture(cap, img);	

	 	/* Image detect process */
	 	for(int i = 0; i < img.size(); i++)
	 	{
  	 	 	// Org Image Landmark
	 	 	std::vector<cv::Point2d> temp;
	 	 	temp = DetectImg(img[i], 0.2, 0.3); // Kr = 0.2, Kb = 0.3
	 	 	Landmark[i] = temp;
	 	}		
	 	/* Warp to 360 pixle coordinate */
	 	warpMark = projectionPoint(camParam_K, camParam_R, warper, shiftCenter, Landmark);
	 	resLandmark = combinePoint(warpMark);//Indentify Landmark
	 	vel =  predvel(total, vel, resLandmark, bufLandmark); // predict pixel rate in horizontal
	 	updateMark = update(vel, resLandmark, bufLandmark); // new pixle 
	 	sortLandmark = sorting(updateMark, 4); // sorting to Landmark close form
 	 	angle = pixelToBearing(sortLandmark);

		/* Localization 4 Landmark */
	 	pos = angleToLocal(angle, pt_real); // position
	 	loc = avgLocal(pos,4); // 4 Landmark Avg Localization
	 	pkg_filt = packagePos(loc, pkg_filt, 10); // package Localization
	 	loc_final = smoother(pkg_filt, total); // smoother filter
											
		/* Log information */
		LOGData(updateMark, vel, angle, loc, loc_final, total);
		std::cout << "---------------------------" << std::endl;

		/* Detect rate */
		for(int i = 0; i < resLandmark.size(); i++)
		{
			if(resLandmark[i].y == 0){
				cnt++;
				break;
			}

		}

		total++;
		std::cout << "total:"<< total << std::endl;


		newFile.precision(9); 
		newFile << loc_final.x << " " << loc_final.y << '\n';

	 	// Image show Visualize
		bool end = VisualizeImg(img, total);
		if(total > 1800)
			break;
	}
	float rate = (1 - (float)cnt / (float)total); //detect rate
	std::cout << "Detect rate:" << rate << std::endl;
	std::cout << "Final the Localization" << std::endl;
	cv::waitKey(0);

	return 0;
}

