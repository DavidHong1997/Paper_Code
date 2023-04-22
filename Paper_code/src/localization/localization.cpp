#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <queue>


using namespace std;
using namespace cv;

double getAngletoVect(Point2d &pt1, Point2d &pt2, Point2d &c)
{
	double theta = atan2(pt1.y - c.y, pt1.x - c.x) - atan2( pt2.y - c.y, pt2.x - c.x);
	if(theta > CV_PI)
		theta -= 2 * CV_PI;
	if(theta < -CV_PI)
		theta += 2 * CV_PI;
	return abs(theta * 180 / CV_PI);
}

Point2d PointToVec(Point2d &pt1, Point2d & pt2)
{
	Point2d Vec(0,0);
	Vec.x = pt1.x - pt2.x;
	Vec.y = pt1.y - pt2.y;
	return Vec;
}



Point2d CircleCenter(double& theta,Point2d Vec,Point2d pt)
{
	Point2d	center;

	center.x = 0.5 * ((cos((2*CV_PI/360) * (90-theta)) * Vec.x) + (-sin((2*CV_PI/360) * (90-theta)) * Vec.y));
	center.y = 0.5 * ((sin((2*CV_PI/360) * (90-theta)) * Vec.x) + ( cos((2*CV_PI/360) * (90-theta)) * Vec.y));

	center.x = center.x * (1/cos((2*CV_PI/360)*(90-theta))) + pt.x;
	center.y = center.y * (1/cos((2*CV_PI/360)*(90-theta))) + pt.y;

	return center;
}

cv::Point2d combCenter(double theta, cv::Point2d vec, cv::Point2d center)
{
	cv::Point2d p;
	p.x = ((cos((CV_PI/180) * (2*theta)) * vec.x) + (-sin((CV_PI/180) * (2*theta)) * vec.y));
	p.y = ((sin((CV_PI/180) * (2*theta)) * vec.x) + ( cos((CV_PI/180) * (2*theta)) * vec.y));
	p = p + center;

	return p;
}

std::vector<cv::Point2d> angleToLocal(std::vector<double>& ang, std::vector<cv::Point2d>& pt)
{
	std::vector<cv::Point2d> res(4);
	cv::Point2d O1, O2;
	cv::Point2d vec_1, vec_2;

	double theta;
	cv::Point2d Vec_op;

	/* Localization 1 */
	vec_1 = PointToVec(pt[2],pt[3]); // pt3 - pt4
	vec_2 = PointToVec(pt[1],pt[2]); // pt2 - pt3
	/* Dual-Circle */
	O1 = CircleCenter(ang[2], vec_1, pt[3]); // Input gamma angle
	O2 = CircleCenter(ang[1], vec_2, pt[2]); // Input beta angle
	
	theta = getAngletoVect( O2, pt[2], O1);
	Vec_op = pt[2] - O1;
	res[0] = combCenter(theta, Vec_op, O1);

	/* Localization 2 */
	vec_1 = PointToVec(pt[1],pt[2]); // pt2 - pt3
	vec_2 = PointToVec(pt[0],pt[1]); // pt1 - pt2
	/* Dual-Circle */
	O1 = CircleCenter(ang[1], vec_1, pt[2]); // Input beta angle
	O2 = CircleCenter(ang[0], vec_2, pt[1]); // Input alpha angle

	theta = getAngletoVect( O2, pt[1], O1);
	Vec_op = pt[1] - O1;
	res[1] = combCenter(theta, Vec_op, O1);

	/* Localization 3 */
	vec_1 = PointToVec(pt[0],pt[1]); // pt1 - pt2
	vec_2 = PointToVec(pt[3],pt[0]); // pt4 - pt1
	/* Dual-Circle */
	O1 = CircleCenter(ang[0], vec_1, pt[1]); // Input alpha angle
	O2 = CircleCenter(ang[3], vec_2, pt[0]); // Input delta angle

	theta = getAngletoVect( O2, pt[0], O1);
	Vec_op = pt[0] - O1;
	res[2] = combCenter(theta, Vec_op, O1);

	/* Localization 4*/
	vec_1 = PointToVec(pt[3],pt[0]); // pt4 - pt1
	vec_2 = PointToVec(pt[2],pt[3]); // pt3 - pt4
	/* Dual-Circle */
	O1 = CircleCenter(ang[3], vec_1, pt[0]); // Input delta angle
	O2 = CircleCenter(ang[2], vec_2, pt[3]); // Input gamma angle

	theta = getAngletoVect( O2, pt[3], O1);
	Vec_op = pt[3] - O1;
	res[3] = combCenter(theta, Vec_op, O1);
	return res;
}

cv::Point2d avgLocal(std::vector<cv::Point2d>& pt, int n)
{
	cv::Point2d ret;
	for(int i = 0; i < pt.size(); i++)
	{
		ret += pt[i];
	}
	ret /= n;
	return ret;
}

// package smoother

std::queue<cv::Point2d> packagePos(cv::Point2d& pos, std::queue<cv::Point2d>& pkg, int n)
{
	if(pkg.size() < n){
		pkg.push(pos);
		return pkg;
	}
	// Update New
	pkg.pop();
	pkg.push(pos);
	return pkg;
}

cv::Point2d smoother(std::queue<cv::Point2d> pkg, int n)
{
	cv::Point2d p;
	if(n > 10){
		while(!pkg.empty()){
			//std::cout << std::fixed << std::setprecision(2) << pkg.front() << std::endl;
			p += pkg.front();	
			pkg.pop();
		}
		p /= 10;
	}
	//std::cout << std::fixed << std::setprecision(2) << p << std::endl;
	return p;
}
