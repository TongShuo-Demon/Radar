////参考链接：https://blog.csdn.net/qq_32925781/article/details/52878465
//https://docs.opencv.org/4.1.0/d1/dc5/tutorial_background_subtraction.html
// 方法：BackgroundSubtractorMOG2
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
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
    VideoCapture capture("120.mp4");

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








//#include <iostream>
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//
//using namespace cv;
//using namespace std;
//
//int main( int argc, char** argv )
//{
//    VideoCapture cap("12mm_red.avi"); //capture the video from web cam
//
//    if ( !cap.isOpened() )  // if not success, exit program
//    {
//        cout << "Cannot open the web cam" << endl;
//        return -1;
//    }
//
//    namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"
//
//    int iLowH = 0;
//    int iHighH = 10;
//
//    int iLowS = 46;
//    int iHighS = 255;
//
//    int iLowV = 46;
//    int iHighV = 255;
//
//    //Create trackbars in "Control" window
//    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
//    createTrackbar("HighH", "Control", &iHighH, 179);
//
//    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//    createTrackbar("HighS", "Control", &iHighS, 255);
//
//    createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
//    createTrackbar("HighV", "Control", &iHighV, 255);
//
//    while (true)
//    {
//        Mat imgOriginal;
//
//        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
//
//        if (!bSuccess) //if not success, break loop
//        {
//            cout << "Cannot read a frame from video stream" << endl;
//            break;
//        }
//
//        Mat imgHSV;
//        vector<Mat> hsvSplit;
//        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
//
//        //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
//        split(imgHSV, hsvSplit);
//        equalizeHist(hsvSplit[2],hsvSplit[2]);
//        merge(hsvSplit,imgHSV);
//        Mat imgThresholded;
//
//        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
//
//        //开操作 (去除一些噪点)
//        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
//        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
//
//        //闭操作 (连接一些连通域)
//        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
//
//        imshow("Thresholded Image", imgThresholded); //show the thresholded image
//        imshow("Original", imgOriginal); //show the original image
//
//        char key = (char) waitKey(300);
//        if(key == 27)
//            break;
//    }
//
//    return 0;
//
//}
