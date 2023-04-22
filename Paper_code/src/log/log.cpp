#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "log.h"


void LOGData(std::vector<cv::Point2d>& lan, std::vector<cv::Point2d>& vel, 
		     std::vector<double>& ang, cv::Point2d& avg, cv::Point2d& filt,int& st)
{
	std::cout << "Landmark pixel" << "         " << "x pred(dx)" << "    " << "Bearing" << std::endl;
	for(int i = 0; i < lan.size(); i++)
	{
		std::cout << std::fixed << std::setprecision(2) 
			<< "x:" << std::right << std::setw(8) << lan[i].x << " " << "y:" << std::right << std::setw(7) << lan[i].y
			<< "   dx:" << std::right << std::setw(6) << vel[i].x
			<< "      "<<std::right << std::setw(6) << ang[i]   
			<< std::endl;
	}
	std::cout << "4 Landmark Avg:" << avg << std::endl;
	if(st > 10){
		std::cout << "Localization:  " << filt << std::endl;
	}
}
