//
// Created by demon on 2019/11/12.
//

#ifndef RADAR_CALCULATE_HPP
#define RADAR_CALCULATE_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <radar_main.hpp>
#include <pretreatment.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>  // Eigen 几何模块
#include <opencv2/core/eigen.hpp>
#define PI 3.1415926

bool limit_rect(cv::Rect &rect, int rows, int cols);
std::string intToString(double number);
cv::Point get_cross_point(cv::Point &p00, cv::Point &p01, cv::Point &p10, cv::Point &p11);
Eigen::Vector3d  cameraToWorld(Eigen::Matrix3d cameraMatrix, Eigen::Matrix3d rv, Eigen::Vector3d tv,cv::Point connor);
Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);

#endif //RADAR_CALCULATE_HPP
