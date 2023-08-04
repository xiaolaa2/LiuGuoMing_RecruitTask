#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using std::placeholders::_1;

class ImageReciever : public rclcpp::Node
{
public:
    ImageReciever() : Node("ImageReciever"), count_(0)
    {
        // 实例化订阅者
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("video_publisher", 10, std::bind(&ImageReciever::subscriber_callback, this, _1));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    size_t count_;

private:
    // 订阅的回调函数
    void subscriber_callback(const sensor_msgs::msg::Image::ConstPtr &msg)// 注意这里的参数类型
    {
        cv::Mat img;
        cv_bridge::CvImageConstPtr cv_ptr; // 获取cv图像的引用指针
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

        img = cv_ptr->image; // 从指针中获取图片

        if (!img.empty())
        {
            detect_face(img);
            cv::imshow("recieve_picture", img); // 显示图片
        }
        else
        {
            // std::cout << "Error load image" << std::endl;
            RCLCPP_INFO(this->get_logger(), "lost");
        }

        char c = (char)cv::waitKey(25); // 每25毫秒读取一次键盘输入
        if (c == 27)
        { // ESC键退出
            exit(0);
        }
    }

    void detect_face(cv::Mat & img) {
        cv::CascadeClassifier faceDetector("./base_task4/asset/haarcascade_frontalface_default.xml");

        //转换为灰度图
        // Mat gray;
        // cvtColor(img, gray, COLOR_BGR2GRAY);

        std::vector<Rect> faces;
        faceDetector.detectMultiScale(img, faces);

        // 遍历所有识别出的脸，画出矩形
        for (auto & face : faces) {
            // Scalar 是RGB格式的颜色
            rectangle(img, face, Scalar(0,255,0), 2);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageReciever>());
    rclcpp::shutdown();
    return 0;
}