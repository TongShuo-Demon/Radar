//
// Created by demon on 2019/11/11.
//
#ifndef RADAR_RADAR_MAIN_HPP
#define RADAR_RADAR_MAIN_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

namespace radar{

    extern int vehicle_min_area;                                  //小车最小面积
    extern int vehicle_max_area;                                  //小车最大面积


    extern cv::Ptr<cv::BackgroundSubtractor> pMOG2;               //MOG2 Background subtractor
    extern cv::Ptr<cv::BackgroundSubtractor> KNN;                 //MOG2 Background subtractor
    extern float vehicle_heigth_div_width_min;
    extern float vehicle_heigth_div_width_max;
    extern float pixel_percent_limit;                             //像素比约束

    void radar_init();
}






#endif //RADAR_RADAR_MAIN_HPP
