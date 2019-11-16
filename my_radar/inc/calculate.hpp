//
// Created by demon on 2019/11/12.
//

#ifndef RADAR_CALCULATE_HPP
#define RADAR_CALCULATE_HPP

#include <iostream>
#include <opencv2/opencv.hpp>


bool limit_rect(cv::Rect &rect, int rows, int cols);
std::string intToString(int number);
cv::Point get_cross_point(cv::Point &p00, cv::Point &p01, cv::Point &p10, cv::Point &p11);



#endif //RADAR_CALCULATE_HPP
