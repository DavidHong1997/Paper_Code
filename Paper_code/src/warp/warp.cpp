#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/stitching/warpers.hpp"
#include "time.h"
#include "tuple"
#include "opencv2/stitching/detail/camera.hpp"
using namespace std;
using namespace cv;
using namespace cv::detail;


std::vector<std::vector<cv::Point2d>> projectionPoint(std::vector<cv::Mat>& K, 
			   std::vector<cv::Mat>& R,
			   Ptr<RotationWarper> warper,
			   const cv::Point &shift,
			   std::vector<std::vector<cv::Point2d>>& pt)
{

	std::vector<std::vector<cv::Point2d>> retPoint(6, std::vector<cv::Point2d>(4));
	for(int i = 0; i < K.size(); i++)
	{
		Mat Kc, Rc;
		K[i].convertTo(Kc, CV_32F);
		R[i].convertTo(Rc, CV_32F);
		for(int j = 0; j < 4; j++)
		{
			//std::cout << "Log Org " << pt[i][j] << std::endl; 
			double x = pt[i][j].x;
			double y = pt[i][j].y;
			if(x!=0 && y!=0){
				cv::Point2d res = warper->warpPoint(pt[i][j], Kc, Rc);
				res.x = res.x - shift.x;
				res.y = res.y - shift.y;
				//std::cout << "Log warp " << res << std::endl;
				retPoint[i][j] = res; // store new panorama result 
			}
		}
		//std::cout << "-----------------------" << std::endl;
	}
	return retPoint;
}

std::vector<cv::Point2d> combinePoint(std::vector<std::vector<cv::Point2d>>& pt)
{
	std::vector<cv::Point2d> res(4);
	for(int i = 0; i < 4; i++)
	{
		double x, y;
		int count = 0;
		cv::Point2d temp;
		for(int j = 0; j < 6; j++)
		{
			x = pt[j][i].x;
			y = pt[j][i].y;
			if(y != 0){
				temp.x = temp.x + x;
				temp.y = temp.y + y;
				count++;
			}
		}

		if(count != 0){
			temp.x = (temp.x / count);
			temp.y = (temp.y / count);
			res[i] = temp;
		}
		//std::cout << temp.x << " " << temp.y << std::endl;
	}
	//std::cout << "----------------split line ------------" << std::endl;
	return res;
}

std::vector<cv::Point2d> predvel(int &cnt,
						   std::vector<cv::Point2d>& vel,
						   std::vector<cv::Point2d>& cur,
						   std::vector<cv::Point2d>& buf)
{
	std::vector<cv::Point2d> res;
	res = vel;
	if(cnt == 0){
		buf = cur;
		return res;
	}

	for(int i = 0; i < 4; i++)
	{
		if(cur[i].x != 0 && cur[i].y != 0){
			res[i].x = cur[i].x - buf[i].x;
			res[i].y = cur[i].y - buf[i].y;

			buf[i].x = cur[i].x; // update buf
			buf[i].y = cur[i].y; // update buf
		}
		//std::cout << res[i] << std::endl; // velocity check point
	}
	return res;
}

std::vector<cv::Point2d> update(std::vector<cv::Point2d>& vel, 
											std::vector<cv::Point2d>& pt, 
											std::vector<cv::Point2d>& buf)
{
	std::vector<cv::Point2d> ret(4);
	ret = pt;
	for(int i = 0; i < 4; i++)
	{
		// update landmark
		if(ret[i].x == 0 && ret[i].y == 0){
			ret[i].x = buf[i].x + vel[i].x;
			ret[i].y = buf[i].y + vel[i].y;	
			// update buf
		}
		//std::cout << ret[i] <<  std::endl;
	}
	return ret;
}

std::vector<cv::Point2d> sorting(std::vector<cv::Point2d>& pt, int n)
{
	// sort the small to high
	std::vector<cv::Point2d> x(4); //x = (x_pixel, index)
	
	for(int i = 0; i < 4; i++) // New package
	{
		x[i].x = pt[i].x; // x_pixel
		x[i].y = i;// index
	}

	for(int i = 0; i < n; i++)
	{
		int min_idx = i;
		for(int j = i + 1; j < n; j++){
			if(x[j].x < x[min_idx].x){
				min_idx = j;
			}
		}
		// swap
		cv::Point2d temp = x[min_idx];
		x[min_idx] = x[i];
		x[i] = temp;
	}
	//std::cout << x << std::endl;
	return x;
}

std::vector<double> pixelToBearing(std::vector<cv::Point2d>& pt)
{
	double cum_ang = 360;
	std::vector<double> ret(4); // (alpha,beta,gamma,delta)
	for(int i = 0; i < 3; i++)
	{
		double diff_x = std::abs(pt[i].x - pt[i+1].x);
		double angle = diff_x / 7321 * 360;
		switch(int(pt[i].y))
		{
			case 0:
				ret[0] = angle; // alpha
				break;
			case 1:
				ret[1] = angle; // beta
				break;
			case 2:
				ret[2] = angle; // gamma
				break;
			case 3:
				ret[3] = angle; // delta
				break;
		}
		cum_ang -= angle;
	}
	ret[pt[3].y] = cum_ang; // close form
	return ret;
}

