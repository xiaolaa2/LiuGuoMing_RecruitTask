#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <unistd.h>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main()
{
    // char cwd[1024];
    // getcwd(cwd, sizeof(cwd));
    // std::cout << cwd;  // 打印工作空间路径
    std::string image_path = "./base_task3/asset/test_image.jpg";
    Mat img = imread(image_path, IMREAD_COLOR);
    std::cout << img.size();

    // 转换为灰度图
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // 高斯滤波,去除噪声影响
    Mat gaus;
    GaussianBlur(gray, gaus, cv::Size(7, 7), 0);

    // 二值化
    // Mat binary;
    // threshold(gray, binary, 110, 255, cv::THRESH_BINARY);

    // Canny边缘检测
    Mat edges;
    Canny(gaus, edges, 50, 150);

    // 画出轮廓
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 遍历轮廓
    for(size_t i = 0; i < contours.size(); i++) {
      
        // 计算面积
        double area = contourArea(contours[i]);
        
        // 面积阈值判断
        if(area > 40) {
          
          // 计算边界框
          Rect box = boundingRect(contours[i]);
          
          // 绘制矩形
          rectangle(img, box, Scalar(0,255,0), 2);
        }
    }

    imshow("Orignal Image", img);
    imshow("Canny Image", edges);

    int k = waitKey(0); 
    return 0;
}