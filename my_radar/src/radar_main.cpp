//
// Created by demon on 2019/11/11.
//
#include <radar_main.hpp>

namespace radar{

     cv::Mat cv_camera_matrix;                       // 相机内参
     cv::Mat cv_dist_coeffs;                         // 畸变系数

     int vehicle_min_area;                           //小车最小面积
     int vehicle_max_area;                           //小车最大面积

    cv::Ptr<cv::BackgroundSubtractor> pMOG2;         //MOG2 Background subtractor
    cv::Ptr<cv::BackgroundSubtractor> KNN;          //MOG2 Background subtractor
    float vehicle_heigth_div_width_min;
    float vehicle_heigth_div_width_max;
    float pixel_percent_limit;
}

void radar::radar_init()
{

    cv::FileStorage file_1("../cfg/radar_cam_para_cfg.yml", cv::FileStorage::READ);
    if(file_1.isOpened())
    {
        std::cout << "try to read radar camera configuration parameter" << std::endl;

        file_1["cameraMatrix"] >> radar::cv_camera_matrix;
        file_1["distCoeffs"] >> radar::cv_dist_coeffs;

        file_1.release();
    }
    else
    {
        std::cerr << "fail to read armor camera configuration parameter" << std::endl;
        return;
    }
    std::cout << "succed to read armor camera configuration parameter" << std::endl;


    cv::FileStorage file_2("../cfg/radar.yml", cv::FileStorage::READ);
    if (file_2.isOpened())
    {
        std::cout << "try to read radar detect configuration parameter" << std::endl;

        radar::vehicle_min_area=(int)file_2["vehicle_min_area"];
        radar::vehicle_max_area=(int)file_2["vehicle_max_area"];
        radar::vehicle_heigth_div_width_min=(float)file_2["vehicle_heigth_div_width_min"];
        radar::vehicle_heigth_div_width_max=(float)file_2["vehicle_heigth_div_width_max"];
        radar::pixel_percent_limit = (float)file_2["pixel_percent_limit"];

        file_2.release();
        radar::pMOG2 = cv::createBackgroundSubtractorMOG2();   //创建 Background Subtractor objects
        radar::KNN = cv::createBackgroundSubtractorKNN();   //创建 Background Subtractor objects
    }else
    {
        std::cerr << "fail to read radar detect configuration parameter" << std::endl;
        return;
    }
    std::cout << "succed to read radar detect configuration parameter" << std::endl;

}