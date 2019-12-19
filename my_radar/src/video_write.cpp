//
// Created by demon on 2019/12/16.
//

#include "video_write.hpp"



ViedoWriter::ViedoWriter(const std::string& mode_name, int space) {
    mode_name_ = mode_name;
    space_ = space;

    time_now_ = boost::posix_time::second_clock::local_time();
    std::string time_now_str_ = to_iso_string(time_now_);
//    std::cout << time_now_str_ << std::endl;

    viedo_dir_name_ = "../video/"+time_now_str_+ "/";
    const char* viedo_dir_name_str = viedo_dir_name_.c_str();
    mkdir(viedo_dir_name_str,S_IRUSR | S_IWUSR | S_IXUSR);//读、写、所有者执行
    viedo_dir_name_ = viedo_dir_name_ + mode_name_ + ".avi";
    writer_.open(viedo_dir_name_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(1280, 1024));

    int cont{0};
    while (!writer_.isOpened()) {
        ++cont;
        writer_.open(viedo_dir_name_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(1280, 1024));
        if (cont >= 10)
            std::cout << "ViedoWriter open failed." << std::endl;
        break;
    }
}

ViedoWriter::~ViedoWriter() = default;

void ViedoWriter::Write(const cv::Mat &image) {
    static int cont{0};
    ++cont;
    if (cont % space_ == 0) {
        cont = 0;
        writer_ << image;
    }
}