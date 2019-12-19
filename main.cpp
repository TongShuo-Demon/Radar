#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <pretreatment.hpp>
#include <radar_main.hpp>
#include <shm.hpp>
#include "video_write.hpp"
#include "switch_function.hpp"


using namespace cv;
using namespace std;

#ifdef write_video
ViedoWriter videowriter("radar", 5);
subscriber camera_sub("camera_pub");  //定义共享内存，共享内存名称需要与预设的相同
#endif

cv::Point2d World2pixel_Coordinate(cv::Point2d world)     //雷达坐标系变换到uwb坐标系下面
{
    std::cout << "world:"<<world<< std::endl;
    cv::Point pixel;
    pixel.x=world.x*30;
    pixel.y=840-world.y*30;

    std::cout << "pixel:"<<pixel << std::endl;
    return pixel;
}




/*

int main()
{
    Mat frame;

    Mat frameCalibration;
    camera_sub.get(frame);

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1.1496e+03;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 603.2387;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 1.1496e+03;
    cameraMatrix.at<double>(1, 2) = 535.4743;
    cameraMatrix.at<double>(2, 0) = 0 ;
    cameraMatrix.at<double>(2, 1) = 0 ;
    cameraMatrix.at<double>(2, 2) = 1;

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.3665;
    distCoeffs.at<double>(1, 0) = 0.1817;
    distCoeffs.at<double>(2, 0) = 0;
    distCoeffs.at<double>(3, 0) = 0;
    distCoeffs.at<double>(4, 0) = -0.0937;
    Mat view, rview, map1, map2;
    Size imageSize;
    imageSize = frame.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                            imageSize, CV_16SC2, map1, map2);


    while (1) //Show the image captured in the window and repeat
    {
        camera_sub.get(frame);        // read
        if (frame.empty()) break;         // check if at end
        remap(frame, frameCalibration, map1, map2, INTER_LINEAR);
        imshow("Origianl", frame);
        imshow("Calibration", frameCalibration);
        char key = waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')break;
    }
    return 0;

}



*/








int main()
{
    namedWindow("Actual_map");
    radar::radar_init();

    cv::Mat src,Actual_map;
    Vehicles vehicles,vehicles1;
    detect_vehicle detectvehicle;
    VideoCapture capture("/home/demon/CLionProjects/Radar/cmake-build-debug/110_70_1.avi");
//    Actual_map=imread("/home/demon/CLionProjects/Radar/cmake-build-debug/Field_map.png");

    if(!capture.isOpened()){
        cout << "Unable to open the video! " << endl;
        exit(EXIT_FAILURE); //EXIT_FAILURE 可以作为exit()的参数来使用，表示没有成功地执行一个程序,其值为1
    }

  while(1){
        if(!capture.read(src)) {
            cout << "Unable to read next frame." << endl;
            exit(0);
        }
      Actual_map=imread("/home/demon/CLionProjects/Radar/cmake-build-debug/Field_map.png");
//        resize(src, src, cv::Size(640,480),0,0,cv::INTER_AREA);

//        long start,end;
//        start = getCurrentTime__();
#ifdef write_video
       camera_sub.get(src);
        imshow("ssss", src);
        videowriter.Write(src);
        waitKey(20);
#endif
        vehicles1=detectvehicle.processVideoKNN(src,vehicles);     //运行时间，大约20多ms
        std::cout << vehicles1.size() << std::endl;
        for(int i=0; i< vehicles1.size();i++)
        {
            if(vehicles1[i].color==RED)
            cv::circle(Actual_map,World2pixel_Coordinate(vehicles1[i].World_Coordinate),
                    7,cv::Scalar(0, 0, 255),-1,8);
            else{
                cv::circle(Actual_map,World2pixel_Coordinate(vehicles1[i].World_Coordinate),
                           7,cv::Scalar(255, 0, 0),-1,8);
            }
        }

        std::cout << Actual_map.size()<< std::endl;
//        resize(Actual_map,Actual_map,Size(450,840));
//        imwrite("Field_map.png",Actual_map);
        imshow("Actual_map",Actual_map);

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
//void cameraToWorld(InputArray cameraMatrix, InputArray rV, InputArray tV, Point2f imgPoints, vector<Point3f> &worldPoints)
//{
//    Mat invK64, invK;
//    invK64 = cameraMatrix.getMat().inv();
//    invK64.convertTo(invK, CV_32F);
//    Mat r, t, rMat;
//    rV.getMat().convertTo(r, CV_32F);
//    tV.getMat().convertTo(t, CV_32F);
//    Rodrigues(r, rMat);
//
//    //计算 invR * T
//    Mat invR = rMat.inv();
    //cout << "invR\n" << invR << endl;
    //cout << "t\n" << t << t.t() << endl;
//    Mat transPlaneToCam;
//    if(t.size() == Size(1, 3)){
//        transPlaneToCam = invR * t;//t.t();
//    }
//    else if(t.size() == Size(3, 1)){
//        transPlaneToCam = invR * t.t();
//    }
//    else{
//        return;
//    }
    //cout << "transPlaneToCam\n" << transPlaneToCam << endl;


    //cout << "npoints\n" << npoints << endl;

//        Mat coords(3, 1, CV_32F);
//        Point3f pt;
//        coords.at<float>(0, 0) = imgPoints.x;
//        coords.at<float>(1, 0) = imgPoints.y;
//        coords.at<float>(2, 0) = 1.0f;
//        //[x,y,z] = invK * [u,v,1]
//        Mat worldPtCam = invK * coords;
//        //cout << "worldPtCam:" << worldPtCam << endl;
//        //[x,y,1] * invR
//        Mat worldPtPlane = invR * worldPtCam;
//        //cout << "worldPtPlane:" << worldPtPlane << endl;
//        //zc
//        float scale = transPlaneToCam.at<float>(2) / worldPtPlane.at<float>(2);
//        //cout << "scale:" << scale << endl;
//        Mat scale_worldPtPlane(3, 1, CV_32F);
//        //scale_worldPtPlane.at<float>(0, 0) = worldPtPlane.at<float>(0, 0) * scale;
//        //zc * [x,y,1] * invR
//        scale_worldPtPlane = scale * worldPtPlane;
//        //cout << "scale_worldPtPlane:" << scale_worldPtPlane << endl;
//        //[X,Y,Z]=zc*[x,y,1]*invR - invR*T
//        Mat worldPtPlaneReproject = scale_worldPtPlane - transPlaneToCam;
//        //cout << "worldPtPlaneReproject:" << worldPtPlaneReproject << endl;
//        pt.x = worldPtPlaneReproject.at<float>(0);
//        pt.y = worldPtPlaneReproject.at<float>(1);
//        //pt.z = worldPtPlaneReproject.at<float>(2);
//        pt.z = 1.0f;
//        worldPoints.push_back(pt);
//    cout<< pt<< endl;
//}


void cameraToWorld(Eigen::Matrix3d cameraMatrix, Eigen::Matrix3d rv, Eigen::Vector3d tv,Eigen::Vector3d pixel,Eigen::Vector3d world)
//void cameraToWorld(Eigen::Matrix3d rv, Eigen::Vector3d tv)
{
    //计算 invR * T
    Eigen::Matrix3d invR;
    Eigen::Vector3d transPlaneToCam,worldPtCam,worldPtPlane,scale_worldPtPlane;
    invR=rv.inverse();
    transPlaneToCam =  invR* tv ;
    //[x,y,z] = invK * [u,v,1]
    worldPtCam = cameraMatrix.inverse()*pixel;
    //[x,y,1] * invR
    worldPtPlane = invR * worldPtCam;
    //zc,存在疑问
    float scale = transPlaneToCam[2] / worldPtPlane[2];
    //zc * [x,y,1] * invR
    scale_worldPtPlane = scale * worldPtPlane;
   //[X,Y,Z]=zc*[x,y,1]*invR - invR*T
    world = scale_worldPtPlane - transPlaneToCam;

    std::cout << world;
}


/*
int main()
{
//    vector<Point3f> worldPoints;
    Eigen::Matrix3d cameraMatrix;
    cameraMatrix << 100.0,0,300.0,
                    0,100.0,200.0,
                     0,0,1;

    Eigen::Matrix3d rv;
                                         rv << 1,0,0,
                                              0,0,1,
                                              0,-1,0;
    Eigen::Vector3d  rt ,pixel,world;

    rt << 0,-3,0;
   pixel << 300.0,230.0,1;
   cameraToWorld(cameraMatrix,rv,rt,pixel,world);

//    cameraToWorld(rv,rt);

}
*/
