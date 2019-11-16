//
// Created by demon on 2019/11/12.
//
#include <calculate.hpp>






//1、描述：
//判断矩形是否在图像内
//2、输入：
//rect：输入矩形
//rows：行界限
//cols：列界限
//3、输出：
//返回：如果在范围内返回true，否则返回flase
bool check_rect(cv::Rect &rect, int rows, int cols)
{
    cv::Rect big_rect(0, 0, cols, rows);
    cv::Rect and_rect;
    and_rect = big_rect & rect;
    if (and_rect.area() < 1)
        return false;

    return true;
}

//1、描述：
//检查待检测值是否在给定范围内，否则更新待值到给定范围内
//2、输入：
//a：待检测值
//hight：范围上限
//low：范围下限
float limit_border(float a, float hight, float low)
{
    if (a > hight)
        return hight;
    if (a < low)
        return low;

    return a;
}



//1、描述：
//首先检查矩形是否在规定范围内
//如果超出范围则去除超出的范围的矩形
//2、输入：
//rect：输入矩形
//rows：行界限
//cols：列界限
//3、输出：
//返回：成功更新矩阵返回true，否则返回false
bool limit_rect(cv::Rect &rect, int rows, int cols)
{
    if (!check_rect(rect, rows, cols))
        return false;
    else
    {
        int x1, y1, x2, y2;
        x1 = limit_border(rect.x, cols - 1, 0);
        y1 = limit_border(rect.y, rows -1 , 0);
        x2 = limit_border(rect.x + rect.width, cols - 1, 0);
        y2 = limit_border(rect.y + rect.height, rows - 1, 0);

        if ((y2 <= y1) || (x2 <= x1))
            return false;

        rect.x = x1;
        rect.y = y1;
        rect.width = x2 - x1;
        rect.height = y2 - y1;

        if ((rect.width < 0) || (rect.height < 0))
            return false;
    }
    return true;
}




std::string intToString(int number)
{
    //this function has a number input and string output
    std::stringstream ss;
    ss << number;
    return ss.str();
}



//p00和p01是一条线段的两个端点
//p10和P11是另一条线段的两个端点
//
cv::Point get_cross_point(cv::Point &p00, cv::Point &p01, cv::Point &p10, cv::Point &p11)
{
    double x_member, x_denominator, y_member, y_denominator;
    cv::Point cross_point;
    x_denominator = p11.x*p01.y - p11.x*p00.y - p10.x*p01.y + p10.x*p00.y
                    - p01.x*p11.y + p01.x*p10.y + p00.x*p11.y - p00.x*p10.y;

    x_member = p10.y*p11.x*p01.x - p11.y*p10.x*p01.x - p10.y*p11.x*p00.x + p11.y*p10.x*p00.x
               - p00.y*p01.x*p11.x + p01.y*p00.x*p11.x + p00.y*p01.x*p10.x - p01.y*p00.x*p10.x;

    if (x_denominator == 0)
        cross_point.x = -1;
    else
        cross_point.x = x_member / x_denominator;

    y_denominator = p11.y*p01.x - p11.y*p00.x - p10.y*p01.x + p00.x*p10.y
                    - p01.y*p11.x + p01.y*p10.x + p00.y*p11.x - p00.y*p10.x;

    y_member = -p10.y*p11.x*p01.y + p11.y*p10.x*p01.y + p10.y*p11.x*p00.y - p11.y*p10.x*p00.y
               + p00.y*p01.x*p11.y - p00.y*p01.x*p10.y - p01.y*p00.x*p11.y + p01.y*p00.x*p10.y;

    if (y_denominator == 0)
        cross_point.y = -1;
    else
        cross_point.y = y_member / y_denominator;

    return cross_point;  //平行返回(0,0)
}




