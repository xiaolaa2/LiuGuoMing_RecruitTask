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
    // 检测到的人脸
    std::vector<Rect> faces;
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

   // 利用引用的方式进行人脸检测并画出对应的矩形和标签
    void detect_face(cv::Mat & img) {
        cv::CascadeClassifier faceDetector;

        // char cwd[1024];
        // getcwd(cwd, sizeof(cwd));
        // std::cout << cwd;  // 打印工作空间路径
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), cwd);

        // 加载特征文件
        faceDetector.load(("./src/base_task4/asset/haarcascade_frontalface_default.xml"));

        if (faceDetector.empty()) {
            return;
        }

        faces.clear();
        faceDetector.detectMultiScale(img, faces); // 检测人脸

        int count = 1;

        // 遍历所有识别出的脸，画出矩形
        for (auto & face : faces) {

            // 绘制标签
            Point origin;
            origin.x = face.x;
            origin.y = face.y - 10; // 稍微偏上一点

            std::string label = "face" + std::to_string(count);
            putText(img, label, origin, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(36,255,12), 2);
            // Scalar 是RGB格式的颜色
            rectangle(img, face, Scalar(0,255,0), 2);

            count++;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 启动服务
    rclcpp::spin(std::make_shared<ImageReciever>());
    rclcpp::shutdown();
    return 0;
}