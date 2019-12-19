//
// Created by demon on 2019/12/16.
//
#ifndef RADAR_VIDEO_WRITE_HPP
#define RADAR_VIDEO_WRITE_HPP

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/greg_month.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/date_formatting.hpp>
#include <string>

class ViedoWriter {
private:
    cv::VideoWriter writer_;
    std::string mode_name_;
    std::string viedo_dir_name_;
    int space_;

    boost::posix_time::ptime time_now_;

public:
    ViedoWriter(const std::string& mode_name, int space);
    ~ViedoWriter();

    void Write(const cv::Mat& image);
};








#endif //RADAR_VIDEO_WRITE_HPP
