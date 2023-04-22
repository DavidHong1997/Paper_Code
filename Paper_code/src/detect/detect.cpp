#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>

#include "detect.h"

int T = 0;


void ImageCapture(std::vector<cv::VideoCapture>& cam,std::vector<cv::Mat>& img)
{
	for(int i = 0; i < cam.size(); i++)
	{
	 	cam[i].read(img[i]);
	}
}


bool VisualizeImg(std::vector<cv::Mat>& img, int cnt)
{
	std::vector<cv::Mat> front,back;
	for(int i = 0; i < img.size(); i++)
	{
	 	cv::Mat temp;

	 	//End Video
	 	if(img[i].empty())
	 	 	return 0;

	 	cv::resize(img[i], temp, cv::Size(img[i].cols/8, img[i].rows/8), 0, 0, cv::INTER_LINEAR);
	 	if(i < 3)
	 	 	front.push_back(temp);
	 	else
	 	 	back.push_back(temp);
	}
	cv::Mat res_f, res_b;
	cv::hconcat(front, res_f);
	cv::hconcat(back, res_b);

	cv::imshow("front", res_f);
	cv::imshow("back", res_b);
	if(cv::waitKey(T)==27){
		if(cnt < 1400)
			T = 60;
		else
			T = 50;
	}

	return 1;
}

std::vector<cv::Point2d> IndetityMark(std::vector<cv::Point2d>& c_r, std::vector<cv::Point2d>& c_b)
{
	double center_x, center_y;
	std::vector<cv::Point2d> Landmark(4); // Landmark[A,B,C,D]
	if(c_r.size() > 0 && c_b.size() > 0){
	 	for(int i = 0; i < c_r.size(); i++)
	 	{
	 	 	for(int j = 0; j < c_b.size(); j++)
	 	 	{
 	 	 	 	double dist_x = c_r[i].x - c_b[j].x;
	 	 	 	double dist_y = c_r[i].y - c_b[j].y;
	 	 	 	double dist = std::sqrt((dist_x * dist_x) + (dist_y * dist_y));
	 	 	 	if(dist < 100){
 	 	 	 	 	center_x = abs(c_r[i].x + c_b[j].x) / 2;
	 	 	 	 	center_y = abs(c_r[i].y + c_b[j].y) / 2;
	 	 	 	 	cv::Point center((int)center_x, (int)center_y);
	 	 	 	 	if(abs(dist_x) < 8 && abs(dist_y) < 70){
	 	 	 	 	 	if(c_r[i].y > c_b[j].y)
	 	 	 	 	 	 	Landmark[3] = center; // Landmark D
	 	 	 	 	 	else if(c_r[i].y < c_b[j].y)
	 	 	 	 	 	 	Landmark[1] = center; // Landmark B
	 	 	 	 	}
 	 	 	 	 	if(abs(dist_y) < 8 && abs(dist_x) < 70){
	 	 	 	 	 	if(c_r[i].x > c_b[j].x)
	 	 	 	 	 	 	Landmark[2] = center; // Landmark C
	 	 	 	 	 	else if(c_r[i].x < c_b[j].x)
	 	 	 	 	 	 	Landmark[0] = center; // Landmark A
	 	 	 	 	}	
	 	 	 	}
	 	 	}
	 	}
	}
	return Landmark;
}

std::vector<cv::Point2d> DetectImg(cv::Mat& img, float Kr, float Kb)
{
	cv::Mat img_ycbcr;
	double cr_max, cr_min;
	double thresh;
	double area;
	cv::Point pt_min, pt_max;
	std::vector<cv::Mat> img_split;

	cv::cvtColor(img, img_ycbcr, cv::COLOR_BGR2YCrCb);
	split(img_ycbcr, img_split);
	minMaxLoc(img_split[1], &cr_min, &cr_max, &pt_min, &pt_max);
	cv::Scalar avg = mean(img_split[1]);
	float cr_avg = avg.val[0];

	std::vector<cv::Point2d> red_center;
	std::vector<cv::Point2d> blue_center;

	if((cr_max - cr_avg) > 30){
	 	thresh = cr_max - (cr_max - cr_avg) * Kr;// 0.2
			
	 	cv::Mat img_thresh_red;
	 	cv::inRange(img_split[1],thresh,255,img_thresh_red);
	
	 	std::vector<std::vector<cv::Point>> contours_r;
	 	std::vector<cv::Vec4i> hierarchy_r;

	 	findContours(img_thresh_red,contours_r, hierarchy_r, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	 	if(contours_r.size() > 0){
	 	 	for(int cnt = 0; cnt < contours_r.size(); cnt++)
	 	 	{
	 	 	 	area = cv::contourArea(contours_r[cnt], false);
	 	 	 	if(area < 100000 && area > 10){
	 	 	 	 	cv::Moments M = cv::moments(contours_r[cnt],false);
	 	 	 	 	double cx = M.m10 / M.m00;
	 	 	 	 	double cy = M.m01 / M.m00;
	 	 	 	 	red_center.push_back(cv::Point2d(cx,cy));
	 	 	 	}
	 	 	}
	 	}
	}

	if((cr_avg - cr_min) > 30){
 	 	thresh = (cr_avg - cr_min) * Kb + cr_min;//0.3
 	 	cv::Mat img_thresh_blue;	
	 	cv::inRange(img_split[1],thresh,255, img_thresh_blue);
		
	 	std::vector<std::vector<cv::Point>> contours_b;
	 	std::vector<cv::Vec4i> hierarchy_b;

	 	findContours(img_thresh_blue,contours_b, hierarchy_b, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	 	if(contours_b.size() > 0){
	 	 	for(int cnt = 0; cnt < contours_b.size(); cnt++)
	 	 	{
	 	 	 	area = cv::contourArea(contours_b[cnt], false);
	 	 	 	if(area < 100000 && area > 10){
	 	 	 	 	cv::Moments M = cv::moments(contours_b[cnt],false);
	 	 	 	 	double cx = M.m10 / M.m00;
	 	 	 	 	double cy = M.m01 / M.m00;
	 	 	 	 	blue_center.push_back(cv::Point2d(cx,cy));
	 	 	 	}
	 	 	}
	 	}
	}
	return IndetityMark(red_center,blue_center);
}

