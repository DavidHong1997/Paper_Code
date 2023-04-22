#ifndef LOCALIZATION_
#define LOCALIZATION_

#include <string>
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>



cv::Point2d pt1(332004.71, 2749887.73);
cv::Point2d pt2(332030.90, 2749902.42);
cv::Point2d pt3(332040.83, 2749885.20);
cv::Point2d pt4(332014.78, 2749870.43);

float getAngletoVec(cv::Point2d &pt1,cv::Point2d &pt2, cv::Point2d &c);

cv::Point2d PointToVec(cv::Point2d &pt1,cv::Point2d &pt2);

cv::Point2d CircleCenter(double& theta,cv::Point2d Vec,cv::Point2d pt);

std::vector<cv::Point2d> angleToLocal(std::vector<double>& , std::vector<cv::Point2d> &);

cv::Point2d avgLocal(std::vector<cv::Point2d>& , int);

std::queue<cv::Point2d> packagePos(cv::Point2d& , std::queue<cv::Point2d>& , int );


cv::Point2d smoother(std::queue<cv::Point2d> , int);

#endif
