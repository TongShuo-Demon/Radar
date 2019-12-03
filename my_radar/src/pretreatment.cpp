//
// Created by demon on 2019/11/11.
//
#include <pretreatment.hpp>

//1、描述
//利用PNP进行单目测距,P3P只能受用四个点
//2、输入
//armor：装甲信息
//3、输出
//返回：测算距离
static double CalDepth(Vehicle vehicle)
{
    //装甲实际宽度，单位毫米
    const float objective_width = 620.0;
    //装甲实际高度，单位毫米
    const float objective_hight = 620.0;

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
    double depth;
    depth = cv_trans_vector.at<double>(2,0);  //x,y,z
    double x_offset= cv_trans_vector.at<double>(0,0);

//    std::cout <<"x_offset:" << x_offset<<std::endl;
//std::cout << cv_trans_vector<<std::endl;
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



// 判断灯条颜色(此函数可以有性能优化).
static int get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) {
//    auto region = blobPos.boundingRect();
//    region.x -= fmax(3, region.width * 0.1);
//    region.y -= fmax(3, region.height * 0.05);
//    region.width += 2 * fmax(3, region.width * 0.1);
//    region.height += 2 * fmax(3, region.height * 0.05);
//    region &= cv::Rect(0, 0, src.cols, src.rows);
//    cv::Mat roi = src(region);
    long red_cnt = 0, blue_cnt = 0;
    for (int row = 0; row < src.rows*0.8; row++) {
        for (int col = 0; col < src.cols*0.8; col++) {
            red_cnt += src.at<cv::Vec3b>(row, col)[2];
            blue_cnt += src.at<cv::Vec3b>(row, col)[0];
        }
    }
    std::cout << red_cnt <<",,"<< blue_cnt<<std::endl;
    if (red_cnt > blue_cnt) {
        return RED;
    } else {
        return BLUE;
    }
}

//判断检测区域的灯条数目
int ROI_find_blob(cv::Mat src_roi,int color) {
    int t = 0;
    cv::Mat color_channel, srcimage;

    std::vector<cv::Mat> channels;       // 通道拆分
    cv::split(src_roi, channels);

    if (color == BLUE) {
        color_channel = channels[0];        /* 根据目标颜色进行通道提取 */
    } else if (color == RED) {
        color_channel = channels[2];
    }
    color_channel.copyTo(srcimage);
    int light_threshold;
    if (color == BLUE) {
        light_threshold = 220;
    } else {
        light_threshold = 200;
    }
    threshold(color_channel, color_channel, light_threshold, 255, cv::THRESH_BINARY);


    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;

    //寻找图像中的轮廓
    cv::findContours(color_channel, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    if (contours.size() > 0) {
        for (int i = 0; i < contours.size(); i++) {
            cv::RotatedRect min_ellipse;                                        //最小外界椭圆
            cv::Rect min_rect;                                                  //最小外接矩形

            min_ellipse = cv::minAreaRect(contours[i]);
            min_rect = min_ellipse.boundingRect();

            float ellipse_h_w_div = min_ellipse.size.height > min_ellipse.size.width ?
                                    min_ellipse.size.height / min_ellipse.size.width :
                                    min_ellipse.size.width / min_ellipse.size.height;
            if (ellipse_h_w_div > 10 || ellipse_h_w_div < 1) {
                continue;
            }

            if (!limit_rect(min_rect, src_roi.rows, src_roi.cols))
                continue;

            //像素进行约束,进行蓝色约束
            cv::Mat bgr_roi, hsv_roi, color_mask_roi;
            bgr_roi = src_roi(min_rect);

            cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);
            if (color == RED) {
                cv::Mat hsv1, hsv2;
                cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
                cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
                color_mask_roi = hsv1 + hsv2;
            } else {
                cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi);
            };

            int correct_pxl = 0;
            for (int j = 0; j < color_mask_roi.rows; j++) {
                auto *hsv_ptr = color_mask_roi.ptr<uchar>(j);
                auto *bgr_ptr = bgr_roi.ptr<uchar>(j);

                for (int k = 0; k < color_mask_roi.cols; k++) {
                    auto hsv_val = hsv_ptr[k];             //第j行，第k列
                    auto b = bgr_ptr[k * 3];
                    auto r = bgr_ptr[k * 3 + 2];

                    if (color == RED) {
                        if (hsv_val && r - b > 30)
                            correct_pxl++;
                    } else {
                        if (hsv_val)
                            correct_pxl++;
                    }

                }
            }
            float pxl_persent = (float) correct_pxl / contourArea(contours[i]);
            if (pxl_persent < 0.1)    //0.08
            {
                continue;
            }

            cv::imshow("eee", color_channel);

            t++;
        }
    }

    return t;
}

bool judge_pxl_persent(cv::Mat bgr_roi,double contours,int color_type)
{
    //对轮廓颜色面积比例进行约束，首先对rgb进行约束，进而对hsv进行约束
                cv::Mat hsv_roi, color_mask_roi;
                cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);
               if(color_type == RED)
               {
                cv::Mat hsv1, hsv2;
                cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
                cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
                color_mask_roi = hsv1 + hsv2;
               } else if(color_type == BLUE){

                cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi);
               };

                int correct_pxl=0,correct_pxl2=0;
                for (int j=0; j < color_mask_roi.rows; j++)
                {
                    auto *hsv_ptr = color_mask_roi.ptr<uchar>(j);
                    auto *bgr_ptr = bgr_roi.ptr<uchar>(j);

                    for (int k = 0; k < color_mask_roi.cols; k++)
                    {
                        auto hsv_val = hsv_ptr[k];             //第j行，第k列
                        auto b = bgr_ptr[k * 3];
                        auto r = bgr_ptr[k * 3 + 2];

                         if(color_type == RED) {
                             if (hsv_val && r - b > 80)
                                 correct_pxl++;
                         } else if(color_type == BLUE)
                         {
                             if(hsv_val)
                                 correct_pxl++;

                         }

                    }
                }
    float pxl_persent = (float)correct_pxl /contours;
//    std::cout <<"像素比约束："<< pxl_persent<<std::endl;
    return pxl_persent < radar::pixel_percent_limit  ;

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
                    std::cout << "面积筛选未通过:"<< contourArea(contours[i]) << std::endl;
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
                    std::cout << "面积比值筛选未通过:" << areaRatio<< std::endl;
                    continue;
                }

                //长宽比值筛选
                float vehicle_heigth_div_width = (float)min_ellipse.boundingRect().height / min_ellipse.boundingRect().width;
                if (radar::vehicle_heigth_div_width_min >  vehicle_heigth_div_width ||
                        vehicle_heigth_div_width > radar::vehicle_heigth_div_width_max ){
                    std::cout << "长宽比值筛选未通过:"<< vehicle_heigth_div_width << std::endl;
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
//                color_type = get_blob_color(bgr_roi,min_ellipse);
            color_type=USE_hsv(bgr_roi);
//                if(judge_pxl_persent(bgr_roi,contourArea(contours[i]),color_type))
//                {
//                    std::cout << "像素筛选未通过" << std::endl;
//                    continue;
//                }

//                if(ROI_find_blob(bgr_roi,color_type) < 2)
//                {
//                    std::cout << "灯条数目筛选未通过" << std::endl;
//                    continue;
//                }
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
                temp_Vehicles[i].sp=temp_sp;  //车中心点
                temp_Vehicles[i].color=color_type; //判断车辆颜色
                temp_Vehicles[i].vehicle_vertex[0]=temp_vertex[0];
                temp_Vehicles[i].vehicle_vertex[1]=temp_vertex[1];
                temp_Vehicles[i].vehicle_vertex[2]=temp_vertex[2];
                temp_Vehicles[i].vehicle_vertex[3]=temp_vertex[3];
                last_sp = temp_sp;

#ifdef show
                cv::imshow("检测到的车辆图",bgr_roi);  //筛选出来的感兴趣区域
                std::cout << "sp_"<< i<<":"<<vehicles[i].sp<<std::endl;
#endif
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

            for(int m=0;m < temp_Vehicles.size();m++)
            {
                int length=CalDepth(temp_Vehicles[m]);
                if( length >2500 ||length < 300 )       //筛选明显不符合条件的
                    continue;
                putText(src, intToString((int)CalDepth(temp_Vehicles[m]))+"cm",
                        cv::Point(temp_Vehicles[m].vehicle_vertex[2].x-10, temp_Vehicles[m].vehicle_vertex[2].y-15),
                     cv::FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,255,0));
//                std::cout << "depth:" << CalDepth(temp_Vehicles[m])<<"cm" << std::endl;
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




int final_shaixuan(int num,Vehicles vehicles)
{
    int number=0;
    if(num == 1)
    {
        return 1;
    }
         for(int m=0;m < num-1;m++)
        {
            for(int n=m+1; n < num;n++)
            {
               if( fabs(vehicles[n].sp.x-vehicles[m].sp.x)>6 &&
                fabs(vehicles[n].sp.y-vehicles[m].sp.y)>6)
               {
                   number++;

               }
            }

        }
return number;

}







