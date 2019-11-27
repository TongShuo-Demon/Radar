
//参考方法：https://www.cnblogs.com/little-monkey/p/7637130.html
// #include "opencv2/opencv.hpp"
//#include<iostream>
//using namespace std;
//using namespace cv;
//
//int CarNum = 0;
//int to string helper function
//string intToString(int number)
//{
//    //this function has a number input and string output
//    stringstream ss;
//    ss << number;
//    return ss.str();
//}

//Mat MoveDetect(Mat frame1, Mat frame2) {
//    Mat result = frame2.clone();
//    Mat gray1, gray2;
//    cvtColor(frame1, gray1, COLOR_BGR2GRAY);
//    cvtColor(frame2, gray2, COLOR_BGR2GRAY);
//
//    Mat diff;
//    absdiff(gray1, gray2, diff);
//    //imshow("absdiss", diff);
//    threshold(diff, diff, 50, 255, THRESH_BINARY);
//    imshow("threshold", diff);
//
//    medianBlur(diff, diff, 5);
//    imshow("medianBlur", diff);
//
//    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
//    Mat element2 = getStructuringElement(MORPH_RECT, Size(50, 50));
//    erode(diff, diff, element);
//    dilate(diff, diff, element2);
//    imshow("dilate", diff);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarcy;
//    findContours(diff, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));//查找轮廓
//    vector<vector<Point>>contours_poly(contours.size());
//    vector<Rect> boundRect(contours.size()); //定义外接矩形集合
//    //drawContours(img2, contours, -1, Scalar(0, 0, 255), 1, 8);  //绘制轮廓
//    int x0 = 0, y0 = 0, w0 = 0, h0 = 0;
//    for (int i = 0; i<contours.size(); i++)
//    {
//        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);//对图像轮廓点进行多边形拟合：轮廓点组成的点集，输出的多边形点集，精度（即两个轮廓点之间的距离），输出多边形是否封闭
//        boundRect[i] = boundingRect(Mat(contours_poly[i]));
//        if (boundRect[i].width>55 && boundRect[i].width<180 && boundRect[i].height>55 && boundRect[i].height<180) {//轮廓筛选
//            x0 = boundRect[i].x;
//            y0 = boundRect[i].y;
//            w0 = boundRect[i].width;
//            h0 = boundRect[i].height;
//
//            rectangle(result, Point(x0, y0), Point(x0 + w0, y0 + h0), Scalar(0, 255, 0), 2, 8, 0);
//            if ((y0 + h0 / 2 + 1) >= 138 && (y0 + h0 / 2 - 1) <= 142) {//经过这条线（区间），车辆数量+1
//                CarNum++;
//            }
//        }
//        line(result, Point(0, 140), Point(568, 140), Scalar(0, 0, 255), 1, 8);//画红线
//        Point org(0, 35);
//        putText(result, "CarNum=" + intToString(CarNum), org, FONT_HERSHEY_SIMPLEX, 0.8f, Scalar(0, 255, 0), 2);
//    }
//    return result;
//}
//
//int main() {
//    VideoCapture cap("out3.avi");
//    if (!cap.isOpened()) //检查打开是否成功
//        return -1;
//    Mat frame;
//    Mat tmp;
//    Mat result;
//    int count = 0;
//    while (1) {
//        cap >> frame;
//        if (frame.empty())//检查视频是否结束
//            break;
//        else {
//            resize(frame,frame,Size(640,480));
//            count++;
//            if (count == 1)
//                result = MoveDetect(frame, frame);
//            else result = MoveDetect(tmp, frame);
//            imshow("video", frame);
//            imshow("result", result);
//            tmp = frame.clone();
//            if (waitKey(20) == 27)
//                break;
//        }
//    }
//    cap.release();
//    return 0;
//}
//
//


//////实现写视频功能
//#include<opencv2/opencv.hpp>
//#include<iostream>
//using namespace cv;
//using namespace std;
//
//int main()
//{
////    'M', 'J', 'P', 'G'　　　　'X','V','I','D'
//    VideoWriter writer("6mm_red.avi",cv::VideoWriter::fourcc('X','V','I','D'),10,Size(1280,1024),true);//Size要和图片尺寸保持一致
//    char filename[50];
//    Mat frame;
//    for (int i = 1; i < 1067; i++)
//    {
//        sprintf(filename,"//home/demon/MVViewer/pictures/%d.bmp",i);
//        frame=imread(filename);
//        if(frame.empty())   break;
//        writer<<frame;
//
//    }
//    cout<<"write end!"<<endl;
//    destroyAllWindows();
//    return 0;
//}


////参考链接：https://blog.csdn.net/qq_32925781/article/details/52878465
////https://docs.opencv.org/4.1.0/d1/dc5/tutorial_background_subtraction.html
//// 方法：BackgroundSubtractorMOG2
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <pretreatment.hpp>
#include <radar_main.hpp>

using namespace cv;
using namespace std;




int main()
{

    radar::radar_init();
    cv::Mat src;
    Vehicles vehicles;
    detect_vehicle detectvehicle;
    VideoCapture capture("6mm_red.avi");

    if(!capture.isOpened()){
        cout << "Unable to open the video! " << endl;
        exit(EXIT_FAILURE); //EXIT_FAILURE 可以作为exit()的参数来使用，表示没有成功地执行一个程序,其值为1
    }
    while(1){
        if(!capture.read(src)) {
            cout << "Unable to read next frame." << endl;
            exit(0);
        }
        resize(src, src, cv::Size(640,480),0,0,cv::INTER_AREA);

//        long start,end;
//        start = getCurrentTime__();
        detectvehicle.processVideoKNN(src,vehicles);     //运行时间，大约20多ms

//        end = getCurrentTime__();
//        std::cout << end-start << std::endl;

        int key;
        key = waitKey(20);
        if (key == 'q' || key == 'Q' || key == 27)
            break;

    }
    destroyAllWindows();
}






















//
//void processVideo() {
//
//    VideoCapture capture("out3.avi"); //参数为0，默认从摄像头读取视频
//
//    if(!capture.isOpened()){
//        cout << "Unable to open the camera! " << endl;
//        exit(EXIT_FAILURE); //EXIT_FAILURE 可以作为exit()的参数来使用，表示没有成功地执行一个程序,其值为1
//    }
//
//    while( true ){
//
//        if(!capture.read(frame)) {
//            cout << "Unable to read next frame." << endl;
//            exit(0);
//        } else{
//
//            ttt++;
//        }
//
//        //对画面进行一定的缩放，方便处理
//        double scale = 1.3;         //缩放比例
//        Mat smallImg(frame.rows / scale,frame.cols / scale,CV_8SC1);
//        resize(frame, frame, Size(640,480),0,0,INTER_AREA);
//
//        pKNN->apply(frame, fgMaskMOG2);    //更新背景模型
//
////        threshold(fgMaskMOG2, fgMaskMOG2, 10, 255, THRESH_BINARY);
//        medianBlur(fgMaskMOG2, fgMaskMOG2, 5);  //中值滤波
//
//        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(6, 6));
//        morphologyEx(fgMaskMOG2, fgMaskMOG2, MORPH_OPEN, element);
//
//
//        frame.copyTo(segm);             //建立一个当前frame的副本
//        findContours(fgMaskMOG2, contours, hierarchy,
//                     RETR_TREE, CHAIN_APPROX_SIMPLE,Point(0,0)); //检测轮廓
//
//        vector <vector<Point> > contours_poly( contours.size());
//        vector <Point2f> center( contours.size());
//        vector <float> radius( contours.size());
//        for( int i = 0; i < contours.size(); i++){
//            //findContours后的轮廓信息contours可能过于复杂不平滑，可以用approxPolyDP函数对该多边形曲线做适当近似
//            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);
//            //得到轮廓的外包络圆
//            minEnclosingCircle( contours_poly[i], center[i], radius[i]);
//
////            std::cout <<"第"<<ttt<<"帧，一共"<<contours.size()<<"轮廓，第"<<i<<"轮廓面积：" << contourArea(contours[i]) <<std::endl;
//            if(contourArea(contours[i]) > 15000)
//            {
//             std::cout <<"第"<<ttt<<"帧，一共"<<contours.size()<<"轮廓，第"<<i<<"轮廓面积：" << contourArea(contours[i]) <<std::endl;
//            }
//        }
//
//        //对所得到的轮廓进行一定的筛选
//        for(int i = 0; i < contours.size(); i++ ){
//            if (500 < contourArea(contours[i]) && contourArea(contours[i]) < 20000 ){
//                circle(segm, center[i], (int)radius[i], Scalar(100, 100, 0), 2, 8, 0);
//                break;
//            }
//        }
//
//        //get the frame number and write it on the current frame
//        stringstream ss;
////        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
////                  cv::Scalar(255,255,255), -1);
//        ss << capture.get(CAP_PROP_POS_FRAMES);
//        string frameNumberString = ss.str();
//        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
//                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//
//        //显示
//        imshow("frame", frame);
//        imshow("Segm", segm);
//        imshow("FG Mask MOG 2", fgMaskMOG2);
//
//        int key;
//        key = waitKey(20);
//        if (key == 'q' || key == 'Q' || key == 27)
//            break;
//    }
//
//    capture.release();
//}

//}

