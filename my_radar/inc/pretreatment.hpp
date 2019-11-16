//
// Created by demon on 2019/11/11.
//

#ifndef RADAR_PRETREATMENT_HPP
#define RADAR_PRETREATMENT_HPP

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <radar_main.hpp>
#include <calculate.hpp>

#define RED    1
#define BLUE   0



/******************* 车类定义　**********************/
class Vehicle{
public:

cv::Point vehicle_vertex[4];      //检测到车辆的四个顶点
cv::Point sp;                    //车辆中心点
int color;                       //车辆颜色

explicit Vehicle(const std::vector<cv::Point> &temp_sps=std::vector<cv::Point>() );
};
typedef std::vector<Vehicle> Vehicles;


int ROI_find_blob(cv::Mat src_roi);

/******************* 检测车类定义　**********************/
class detect_vehicle
{
private:
    int num=0;

public:
    Vehicles vehicles;
    void processVideoMOG2(cv::Mat src);
    Vehicles processVideoKNN(cv::Mat src,Vehicles vehicles);

};


















#endif //RADAR_PRETREATMENT_HPP
