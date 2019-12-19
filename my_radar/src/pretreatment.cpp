//
// Created by demon on 2019/11/11.
//
#include <pretreatment.hpp>
#include <calculate.hpp>



//-75*PI/180
Eigen::Vector3d pixel_world(cv::Point connor,double angle)
{
    double Radian = angle*PI/180;              //角度变成弧度
    Eigen::Vector3d pixel,radar_L,world;
    pixel << connor.x, connor.y, 1;  //像素坐标
    Eigen::Matrix3d neican ;
    neican << 1130.6,0,608.2,
              0,1130.6,529.1,
              0,0,1;
//    radar_L << (double)3.6*(608.2-pixel[0])/(pixel[1]-529.1), (double)3.6*1130.7/(pixel[1]-529.1), 0;
    radar_L << (double)-3.6*(pixel[0]-608.2)*sin(Radian)/(1130.6*cos(Radian )-sin(Radian)*(pixel[1]-529.1)),
            (double)3.6*1130.6 /  (1130.6*cos(Radian )-sin(Radian )*(pixel[1]-529.1)),
               0;
    world << radar_L[0]+2,radar_L[1],0;
//    std::cout << "坐标：" << world<< std::endl;
    return world;
}


//1、描述
//利用PNP进行单目测距
//2、输入
//armor：装甲信息
//3、输出
//返回：测算距离
static double CalDepth(Vehicle vehicle)
{
    //装甲实际宽度，单位毫米
    const float objective_width = 600.0;
    //装甲实际高度，单位毫米
    const float objective_hight = 600.0;

    //以装甲中心为世界世界坐标系原点，得到四个角点的世界坐标
    std::vector<cv::Point3f> object_points;
    object_points.emplace_back(-objective_width*0.5, -objective_hight*0.5, 20.0);
    object_points.emplace_back( objective_width*0.5, -objective_hight*0.5, 20.0);
    object_points.emplace_back( objective_width*0.5,  objective_hight*0.5, 20.0);
    object_points.emplace_back(-objective_width*0.5,  objective_hight*0.5, 20.0);

    //装甲四个角点的像素坐标
    std::vector<cv::Point2f> pixel_points;
    pixel_points.emplace_back(vehicle.vehicle_vertex[0]);
    pixel_points.emplace_back(vehicle.vehicle_vertex[1]);
    pixel_points.emplace_back(vehicle.vehicle_vertex[2]);
    pixel_points.emplace_back(vehicle.vehicle_vertex[3]);

    cv::Mat cv_rot_vector(3, 1, CV_64F);  //采用双精度
    cv::Mat cv_trans_vector(3, 1, CV_64F);

    cv::solvePnP(object_points, pixel_points, radar::cv_camera_matrix, radar::cv_dist_coeffs, cv_rot_vector, cv_trans_vector);
//      cv::solvePnPRansac(object_points, pixel_points, radar::cv_camera_matrix, radar::cv_dist_coeffs, cv_rot_vector, cv_trans_vector,
//      true,1000,1,0.8);
    double depth_z,depth_y,depth_x,depth;
    depth_z = cv_trans_vector.at<double>(2,0);  //x,y,z
//    depth_y = cv_trans_vector.at<double>(1,0);  //x,y,z
//    depth_x = cv_trans_vector.at<double>(0,0);  //x,y,z
//
//    double x_offset= cv_trans_vector.at<double>(0,0);
//    depth = pow((pow(depth_z,2)+pow(depth_y,2)),0.5);
//    depth = pow((pow(depth,2)+pow(depth_x,2)),0.5);
// std::cout <<"x_offset:" << x_offset<<std::endl;
//std::cout << cv_trans_vector<<std::endl;
depth =sqrt(depth_z*depth_z-3000*3000);
    return depth / 10;
}




//围绕矩形中心缩放
cv::Rect rectCenterScale(cv::Rect rect, cv::Size size)
{
    rect = rect + size;
    cv::Point pt;
    pt.x = cvRound(size.width/2.0);  //返回跟参数最接近的整数值，即四舍五入；
    pt.y = cvRound(size.height/2.0);
    return (rect-pt);
}



long getCurrentTime__()           //秒和微秒
{
    struct timeval tv;
    gettimeofday(&tv,NULL);     //从1970到现在的时间
    return tv.tv_sec*1000000+tv.tv_usec;
}

static int USE_hsv(const cv::Mat &bgr_roi)
{
    //对轮廓颜色面积比例进行约束，首先对rgb进行约束，进而对hsv进行约束
    cv::Mat hsv_roi, color_mask_roi1,color_mask_roi2;
    cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);

        cv::Mat hsv1, hsv2;
        cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
        cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
        color_mask_roi1 = hsv1 + hsv2;

        cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi2);

    int correct_pxl1=0,correct_pxl2=0;

    for (int j=0; j < color_mask_roi1.rows; j++)
    {
        auto *hsv_ptr1 = color_mask_roi1.ptr<uchar>(j);
        auto *bgr_ptr1 = bgr_roi.ptr<uchar>(j);
        for (int k = 0; k < color_mask_roi1.cols; k++)
        {
            auto hsv_val1 = hsv_ptr1[k];             //第j行，第k列
            auto b1 = bgr_ptr1[k * 3];
            auto r1 = bgr_ptr1[k * 3 + 2];

                if (hsv_val1 && r1 - b1 > 0)
                    correct_pxl1++;

        }
    }

    for (int j=0; j < color_mask_roi2.rows; j++)
    {
        auto *hsv_ptr2 = color_mask_roi2.ptr<uchar>(j);
        auto *bgr_ptr2 = bgr_roi.ptr<uchar>(j);
        for (int k = 0; k < color_mask_roi2.cols; k++)
        {
            auto hsv_val2 = hsv_ptr2[k];             //第j行，第k列
            auto b2 = bgr_ptr2[k * 3];
            auto r2 = bgr_ptr2[k * 3 + 2];
                if(hsv_val2)
                    correct_pxl2++;


        }
    }
return correct_pxl1 > correct_pxl2;
}






//检测车辆
Vehicles detect_vehicle::processVideoKNN(cv::Mat src,Vehicles vehicles)  {
    cv::Mat fgMaskNN; //通过knn方法得到的掩码图像fgmask
    cv::Mat segm;      //frame的副本
    std::vector<std::vector<cv::Point> > contours;  //轮廓
    std::vector<cv::Vec4i> hierarchy;               //轮廓等级关系
    radar::KNN->apply(src, fgMaskNN);    //更新背景模型

    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20));
    morphologyEx(fgMaskNN, fgMaskNN, cv::MORPH_ERODE, element);
    morphologyEx(fgMaskNN, fgMaskNN, cv::MORPH_DILATE, element2);

    threshold(fgMaskNN, fgMaskNN, 2, 255, cv::THRESH_OTSU);//THRESH_BINARY
    medianBlur(fgMaskNN, fgMaskNN, 9);  //中值滤波

    src.copyTo(segm);             //建立一个当前frame的副本
    findContours(fgMaskNN, contours, hierarchy,
                 cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,cv::Point(0,0)); //检测最外围轮廓

    cv::RotatedRect min_ellipse;

    num++;
    if (contours.size() > 0)
    {
        Vehicles temp_Vehicles(contours.size()); //多少个车辆
//        std::cout << "检测的轮廓数量为"<< contours.size() <<std::endl;
        for (int i = 0; i < contours.size(); i++) {

                min_ellipse = fitEllipse(contours[i]);                  //椭圆拟合
                //面积筛选
                if (radar::vehicle_min_area > contourArea(contours[i]) ||
                        contourArea(contours[i]) > radar::vehicle_max_area ){
//                    std::cout << "面积筛选未通过:"<< contourArea(contours[i]) << std::endl;
                    continue;
                }
#ifdef show
                std::cout << "contourArea(contours[i]):" <<contourArea(contours[i])<< std::endl;
                std::cout << " min_ellipse:" << min_ellipse.boundingRect().area()<< std::endl;
                std::cout << ".............." << std::endl;
#endif
                //轮廓面积和外接矩形面积筛选
                float areaRatio = contourArea(contours[i])/min_ellipse.boundingRect().area();
                if (areaRatio > 0.7 || areaRatio < 0.3 ){
//                    std::cout << "面积比值筛选未通过:" << areaRatio<< std::endl;
                    continue;
                }

                //长宽比值筛选
                float vehicle_heigth_div_width = (float)min_ellipse.boundingRect().height / min_ellipse.boundingRect().width;
                if (radar::vehicle_heigth_div_width_min >  vehicle_heigth_div_width ||
                        vehicle_heigth_div_width > radar::vehicle_heigth_div_width_max ){
//                    std::cout << "长宽比值筛选未通过:"<< vehicle_heigth_div_width << std::endl;
                    continue;
                }

                cv::Rect min_rect;
                min_rect=rectCenterScale(min_ellipse.boundingRect(),cv::Size(-20,-20));

//                min_rect = min_ellipse.boundingRect();
                //限制轮廓外接矩形roi在图像内部
                if (!limit_rect( min_rect, src.rows, src.cols ))
                    continue;

                cv::Mat bgr_roi;
                bgr_roi = src(min_rect);
                int color_type=-1;
                color_type=USE_hsv(bgr_roi);

                cv::Mat bgr_roi_out;
//                cv::resize(bgr_roi,bgr_roi_out,cv::Size(640,480));
                //求出中心点
                cv::Point temp_vertex[4],temp_sp,last_sp;
                temp_vertex[0] = min_ellipse.boundingRect().tl(); //左上角
                temp_vertex[1].x = min_ellipse.boundingRect().tl().x + min_ellipse.boundingRect().width;
                temp_vertex[1].y = min_ellipse.boundingRect().tl().y;
                temp_vertex[2] = min_ellipse.boundingRect().br();
                temp_vertex[3].x = min_ellipse.boundingRect().br().x - min_ellipse.boundingRect().width;
                temp_vertex[3].y = min_ellipse.boundingRect().br().y;
                temp_sp=get_cross_point( temp_vertex[0],temp_vertex[2],temp_vertex[1],temp_vertex[3]);
                if(fabs(temp_sp.x-last_sp.x) < 40 && fabs(temp_sp.y-last_sp.y) < 40)
                {
                    temp_sp.x=fabs(temp_sp.x-last_sp.x)/2 + (last_sp.x > temp_sp.x ? last_sp.x:temp_sp.x);
                    temp_sp.y=fabs(temp_sp.y-last_sp.y)/2 + (last_sp.y > temp_sp.y ? last_sp.y:temp_sp.y);
                    temp_Vehicles[i].sp=temp_sp;  //车中心点
//                    std::cout << "temp_sp.x=" <<fabs(temp_sp.x-last_sp.x)<<"  temp_sp.y=" <<fabs(temp_sp.y-last_sp.y)<<std::endl;
                    continue;
                }
            Eigen::Matrix3d cameraMatrix,rv;
            cameraMatrix << 1130.6,0,608.2,
                    0,1130.6,529.1,
                    0,0,1;
            Eigen::Vector3d theta;
            theta << -65,0,9;
            rv=eulerAnglesToRotationMatrix(theta);
            std::cout<< "rv:" << rv << std::endl;
            Eigen::Vector3d  rt ,pixel;
            rt << 0,-3.6,0;

                temp_Vehicles[i].sp=temp_sp;  //车中心点
                temp_Vehicles[i].color=color_type; //判断车辆颜色
                temp_Vehicles[i].vehicle_vertex[0]=temp_vertex[0];
                temp_Vehicles[i].vehicle_vertex[1]=temp_vertex[1];
                temp_Vehicles[i].vehicle_vertex[2]=temp_vertex[2];
                temp_Vehicles[i].vehicle_vertex[3]=temp_vertex[3];
                last_sp = temp_sp;
                temp_Vehicles[i].color = color_type;
//              temp_Vehicles[i].World_Coordinate.x=pixel_world(temp_Vehicles[i].sp,-68)[0];
//              temp_Vehicles[i].World_Coordinate.y=pixel_world(temp_Vehicles[i].sp,-68)[1];
            temp_Vehicles[i].World_Coordinate.x=cameraToWorld(cameraMatrix,rv,rt,temp_Vehicles[i].sp)[0];
            temp_Vehicles[i].World_Coordinate.y=cameraToWorld(cameraMatrix,rv,rt,temp_Vehicles[i].sp)[1];
#ifdef show
                cv::imshow("检测到的车辆图",bgr_roi);  //筛选出来的感兴趣区域
                std::cout << "sp_"<< i<<":"<<vehicles[i].sp<<std::endl;
#endif

            for(int m=0;m < temp_Vehicles.size();m++)
            {
                int length=CalDepth(temp_Vehicles[m]);
                if( length >2500 ||length < 300 )       //筛选明显不符合条件的
                    continue;

                if(color_type==1)
                {
                    putText(src, "RED", cv::Point(temp_sp.x+10, temp_sp.y+15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,255));
                }else
                {
                    putText(src, "BLUE", cv::Point(temp_sp.x+10, temp_sp.y+15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
                }
                cv::circle(src,temp_sp,2,cv::Scalar(255, 0, 0),-1,8);
                intToString(num);
                putText(src, intToString(num), cv::Point(15, 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));

                cv::rectangle(src,min_rect,cv::Scalar(0, 0, 255), 2, 8, 0);
                ;

                putText(src, intToString(temp_Vehicles[m].World_Coordinate.y)+"m,x="+intToString(temp_Vehicles[m].World_Coordinate.x),
                        cv::Point(temp_Vehicles[m].vehicle_vertex[2].x-10, temp_Vehicles[m].vehicle_vertex[2].y-15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,255,0));

                vehicles.emplace_back(temp_Vehicles[m]); //所有通过筛选的存储起来
            }
        }
//       std::cout << "当前帧检测到的车辆数量为："<< vehicles.size()<< std::endl;
    }
     //显示
    imshow("Segm", segm);
    imshow("fgMaskNN", fgMaskNN);
    imshow("src", src);

    return vehicles;
}








