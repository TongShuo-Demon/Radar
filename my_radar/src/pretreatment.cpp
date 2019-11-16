//
// Created by demon on 2019/11/11.
//
#include <pretreatment.hpp>

Vehicle::Vehicle(const std::vector<cv::Point> &temp_sps) {}





//利用MOG2检测方法
void detect_vehicle::processVideoMOG2(cv::Mat src)  {
    cv::Mat fgMaskMOG2; //通过MOG2方法得到的掩码图像fgmask
    cv::Mat segm;      //frame的副本

    std::vector<std::vector<cv::Point> > contours;  //轮廓
    std::vector<cv::Vec4i> hierarchy;               //轮廓等级关系

    //最后一个参数插值方法，是默认值，放大时最好选 INTER_LINEAR ，缩小时最好选 INTER_AREA。
    resize(src, src, cv::Size(640,480),0,0,cv::INTER_AREA);


    radar::pMOG2->apply(src, fgMaskMOG2);    //更新背景模型

    src.copyTo(segm);             //建立一个当前frame的副本
    findContours(fgMaskMOG2, contours, hierarchy,
                 cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,cv::Point(0,0)); //检测轮廓

    std::vector <std::vector<cv::Point> > contours_poly( contours.size());
    std::vector <cv::Point2f> center( contours.size());
    std::vector <float> radius( contours.size());

    for( int i = 0; i < contours.size(); i++){
        //findContours后的轮廓信息contours可能过于复杂不平滑，可以用approxPolyDP函数对该多边形曲线做适当近似
        approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true);
        //得到轮廓的外包络圆
        minEnclosingCircle( contours_poly[i], center[i], radius[i]);
//      std::cout <<"第"<<ttt<<"帧，一共"<<contours.size()<<"轮廓，第"<<i<<"轮廓面积：" << contourArea(contours[i]) <<std::endl;
    }

    //对所得到的轮廓进行面积筛选
    for(int i = 0; i < contours.size(); i++ ){
        if (radar::vehicle_min_area < contourArea(contours[i]) &&
            contourArea(contours[i]) < radar::vehicle_max_area ){
            circle(segm, center[i], (int)radius[i], cv::Scalar(100, 100, 0), 2, 8, 0);
            break;
        }
    }
    //显示
    imshow("frame", src);
    imshow("Segm", segm);
    imshow("FG Mask MOG 2", fgMaskMOG2);
}




//利用KNN检测方法
//void detect_vehicle::processVideoKNN(cv::Mat src)  {
//    cv::Mat fgMaskNN; //通过MOG2方法得到的掩码图像fgmask
//    cv::Mat segm;      //frame的副本
//
//    std::vector<std::vector<cv::Point> > contours;  //轮廓
//    std::vector<cv::Vec4i> hierarchy;               //轮廓等级关系
//
//    //最后一个参数插值方法，是默认值，放大时最好选 INTER_LINEAR ，缩小时最好选 INTER_AREA。
//    resize(src, src, cv::Size(640,480),0,0,cv::INTER_AREA);
//
//    radar::KNN->apply(src, fgMaskNN);    //更新背景模型
//    medianBlur(fgMaskNN, fgMaskNN, 5);  //中值滤波
//    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
//    morphologyEx(fgMaskNN, fgMaskNN, cv::MORPH_OPEN, element);
//    src.copyTo(segm);             //建立一个当前frame的副本
//    findContours(fgMaskNN, contours, hierarchy,
//                 cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,cv::Point(0,0)); //检测最外围轮廓
//
//    std::vector <std::vector<cv::Point> > contours_poly( contours.size());
//    std::vector <cv::Point2f> center( contours.size());
//    std::vector <float> radius( contours.size());
//
//    for( int i = 0; i < contours.size(); i++){
//        //findContours后的轮廓信息contours可能过于复杂不平滑，可以用approxPolyDP函数对该多边形曲线做适当近似
//        approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true);
//        //得到轮廓的外包络圆
//        minEnclosingCircle( contours_poly[i], center[i], radius[i]);
//    }
//
//    //对所得到的轮廓进行面积筛选
//    for(int i = 0; i < contours.size(); i++ ){
//        if (radar::vehicle_min_area > contourArea(contours[i]) &&
//            contourArea(contours[i]) > radar::vehicle_max_area ){
//            circle(segm, center[i], (int)radius[i], cv::Scalar(100, 100, 0), 2, 8, 0);
//            break;
//        }
//
//    }
////    rectangle(src, cv::Point(10, 2), cv::Point(100,20),
////                  cv::Scalar(0,0,255), -1);
//
//    //显示
//    imshow("frame", src);
//    imshow("Segm", segm);
//    imshow("fgMaskNN", fgMaskNN);
//}

// 判断灯条颜色(此函数可以有性能优化).
static int get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) {
//    auto region = blobPos.boundingRect();
//    region.x -= fmax(3, region.width * 0.1);
//    region.y -= fmax(3, region.height * 0.05);
//    region.width += 2 * fmax(3, region.width * 0.1);
//    region.height += 2 * fmax(3, region.height * 0.05);
//    region &= cv::Rect(0, 0, src.cols, src.rows);
//    cv::Mat roi = src(region);
    int red_cnt = 0, blue_cnt = 0;
    for (int row = 0; row < src.rows; row++) {
        for (int col = 0; col < src.cols; col++) {
            red_cnt += src.at<cv::Vec3b>(row, col)[2];
            blue_cnt += src.at<cv::Vec3b>(row, col)[0];
        }
    }
    if (red_cnt > blue_cnt) {
        return RED;
    } else {
        return BLUE;
    }
}

int ROI_find_blob(cv::Mat src_roi,int color)
{
    int t=0;
    cv::Mat color_channel,srcimage;


    std::vector<cv::Mat> channels;       // 通道拆分
    cv::split(src_roi, channels);

    if (color == BLUE) {         /*                      */
        color_channel = channels[0];        /* 根据目标颜色进行通道提取 */
    } else if (color == RED) {    /*                      */
        color_channel = channels[2];        /************************/
    }
    color_channel.copyTo(srcimage);
    int light_threshold;
    if(color == BLUE){
        light_threshold = 220;
    }else{
        light_threshold = 200;
    }
    threshold(color_channel, color_channel, light_threshold, 255, cv::THRESH_BINARY);


    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;

    //寻找图像中的轮廓
    cv::findContours(color_channel, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            cv::RotatedRect min_ellipse;                                        //最小外界椭圆
            cv::Rect min_rect;                                                  //最小外接矩形

            if (contours[i].size() <= 2)
                continue;
            else if (contours[i].size() > 2) {
                min_ellipse = cv::minAreaRect(contours[i]);
                min_rect = min_ellipse.boundingRect();

                float ellipse_h_w_div = min_ellipse.size.height > min_ellipse.size.width ?
                                        min_ellipse.size.height / min_ellipse.size.width :
                                        min_ellipse.size.width / min_ellipse.size.height;
                if(ellipse_h_w_div > 10 || ellipse_h_w_div < 1)
                {
                    continue;
                }

                if (!limit_rect( min_rect, src_roi.rows, src_roi.cols ))
                    continue;

                //像素进行约束,进行蓝色约束
                cv::Mat bgr_roi, hsv_roi, color_mask_roi;
                bgr_roi = src_roi(min_rect);


                cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);
                if(color == RED)
                {
                    cv::Mat hsv1, hsv2;
                    cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
                    cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
                    color_mask_roi = hsv1 + hsv2;
                } else{
                    cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi);
                };

                int correct_pxl=0;
                for (int j=0; j < color_mask_roi.rows; j++)
                {
                    auto *hsv_ptr = color_mask_roi.ptr<uchar>(j);
                    auto *bgr_ptr = bgr_roi.ptr<uchar>(j);

                    for (int k = 0; k < color_mask_roi.cols; k++)
                    {
                        auto hsv_val = hsv_ptr[k];             //第j行，第k列
                        auto b = bgr_ptr[k * 3];
                        auto r = bgr_ptr[k * 3 + 2];

                        if(color == RED) {
                            if (hsv_val && r - b > 30)
                                correct_pxl++;
                        } else
                        {
                            if(hsv_val)
                                correct_pxl++;

                        }

                    }
                }
                float pxl_persent = (float)correct_pxl / contourArea(contours[i]);
                if (pxl_persent < 0.09)    //0.08
                {
                    continue;
                }

                cv::imshow("eee",color_channel);

                t++;
            }
        }

    }
    return t;
}



Vehicles detect_vehicle::processVideoKNN(cv::Mat src,Vehicles vehicles)  {
    cv::Mat fgMaskNN; //通过knn方法得到的掩码图像fgmask
    cv::Mat segm;      //frame的副本

   std::vector<cv::Point> sps;

    std::vector<std::vector<cv::Point> > contours;  //轮廓
    std::vector<cv::Vec4i> hierarchy;               //轮廓等级关系

    radar::KNN->apply(src, fgMaskNN);    //更新背景模型
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6));
    cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(12, 10));
        morphologyEx(fgMaskNN, fgMaskNN, cv::MORPH_ERODE, element);
        morphologyEx(fgMaskNN, fgMaskNN, cv::MORPH_DILATE, element2);

    threshold(fgMaskNN, fgMaskNN, 8, 255, cv::THRESH_BINARY);
    medianBlur(fgMaskNN, fgMaskNN, 7);  //中值滤波

    src.copyTo(segm);             //建立一个当前frame的副本
    findContours(fgMaskNN, contours, hierarchy,
                 cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,cv::Point(0,0)); //检测最外围轮廓

       cv::RotatedRect min_ellipse;

    num++;
    if (contours.size() > 0)
    {
        Vehicles temp_Vehicles(contours.size()); //多少个车辆
        for (int i = 0; i < contours.size(); i++) {
            if (contours[i].size() <= 10)         //第i个轮廓的所有像素点数目
                continue;
            else if (contours[i].size() > 10) {
                min_ellipse = fitEllipse(contours[i]);                  //椭圆拟合
                //面积筛选
                if (radar::vehicle_min_area > contourArea(contours[i]) ||
                        contourArea(contours[i]) > radar::vehicle_max_area ){
                    std::cout << "面积筛选未通过" << std::endl;
                    continue;
                }
//                std::cout << "contourArea(contours[i]):" <<contourArea(contours[i])<< std::endl;
//                std::cout << " min_ellipse:" << min_ellipse.boundingRect().area()<< std::endl;
//                std::cout << ".............." << std::endl;

                //轮廓面积和外接矩形面积筛选
                float areaRatio = contourArea(contours[i])/min_ellipse.boundingRect().area();
                if (areaRatio > 0.9 ||
                        areaRatio < 0.3 ){
                    std::cout << "面积比值筛选未通过" << std::endl;
                    continue;
                }

                //长宽比值筛选
                float vehicle_heigth_div_width = (float)min_ellipse.boundingRect().height / min_ellipse.boundingRect().width;
                if (radar::vehicle_heigth_div_width_min >  vehicle_heigth_div_width ||
                        vehicle_heigth_div_width > radar::vehicle_heigth_div_width_max ){
                    std::cout << "长宽比值筛选未通过" << std::endl;
                    continue;
                }

                cv::Rect min_rect;
                min_rect = min_ellipse.boundingRect();
                //限制轮廓外接矩形roi在图像内部
                if (!limit_rect( min_rect, src.rows, src.cols ))
                    continue;

                //对轮廓颜色面积比例进行约束，首先对rgb进行约束，进而对hsv进行约束
                cv::Mat bgr_roi, hsv_roi, color_mask_roi,throsld;
                bgr_roi = src(min_rect);
                bgr_roi.copyTo(throsld);

                int color_type=-1;
                color_type = get_blob_color(bgr_roi,min_ellipse);

                cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);
               if(color_type == RED)
               {
                   putText(src, "RED", cv::Point(500, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1 , cv::Scalar(0,0,255));
                cv::Mat hsv1, hsv2;
                cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
                cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
                color_mask_roi = hsv1 + hsv2;
               } else if(color_type == BLUE){
                   putText(src, "BLUE", cv::Point(500, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1 , cv::Scalar(255,0,0));
                           cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi);
               };

                int correct_pxl=0;
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
                             if (hsv_val && r - b > 30)
                                 correct_pxl++;
                         } else if(color_type == BLUE)
                         {
                             if(hsv_val)
                                 correct_pxl++;

                         }

                    }
                }
                float pxl_persent = (float)correct_pxl / contourArea(contours[i]);
                if (pxl_persent < radar::pixel_percent_limit)    //0.08
                {
                    std::cout << "像素约束筛选未通过" << std::endl;
                    continue;
                }
                if(ROI_find_blob(bgr_roi,color_type) < 3)
                {
                    std::cout << "灯条数目筛选未通过" << std::endl;
                    continue;
                }

                //求出中心点
                cv::Point temp_vertex[4],temp_sp;
                temp_vertex[0] = min_ellipse.boundingRect().tl(); //左上角
                temp_vertex[1].x = min_ellipse.boundingRect().tl().x + min_ellipse.boundingRect().width;
                temp_vertex[1].y = min_ellipse.boundingRect().tl().y;
                temp_vertex[2] = min_ellipse.boundingRect().br();
                temp_vertex[3].x = min_ellipse.boundingRect().br().x - min_ellipse.boundingRect().width;
                temp_vertex[3].y = min_ellipse.boundingRect().br().y;
                temp_sp=get_cross_point( temp_vertex[0],temp_vertex[2],temp_vertex[1],temp_vertex[3]);

                temp_Vehicles[i].sp=temp_sp;  //车中心点

                cv::imshow("检测到的车辆图",bgr_roi);  //筛选出来的感兴趣区域

//               std::cout << "sp_"<< i<<":"<<vehicles[i].sp<<std::endl;
                 cv::circle(src,temp_sp,8,cv::Scalar(255, 0, 0),-1,8);

                intToString(num);
                putText(src, intToString(num), cv::Point(15, 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
                cv::rectangle(src,min_ellipse.boundingRect(),cv::Scalar(0, 0, 255), 3, 8, 0);


                imshow("frame", src);
            }

        }
        for(int m=0;m < temp_Vehicles.size();m++)
        {
            vehicles.emplace_back(temp_Vehicles[m]); //所有通过筛选的存储起来
        }

    }
     //显示
    imshow("Segm", segm);
    imshow("fgMaskNN", fgMaskNN);
return vehicles;
}












