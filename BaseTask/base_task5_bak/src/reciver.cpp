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
#include "custom_interface/srv/object_position.hpp"

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

    // 获得检测到的物品位置
    void getPosition(const std::shared_ptr<custom_interface::srv::ObjectPosition::Request> request,
                     std::shared_ptr<custom_interface::srv::ObjectPosition::Response> response)
    {
        // 以数组的形式返回检测到的物品的x,y,和名字
        std::vector<int> object_x;
        std::vector<int> object_y;
        std::vector<std::string> name;

        for (size_t i = 0; i < faces.size(); i++) {
            object_x.emplace_back(faces[i].x);
            object_y.emplace_back(faces[i].y);
            name.emplace_back("face" + std::to_string(i+1));
        }

        // 设置返回体的内容
        response->x = object_x;
        response->y = object_y;
        response->name = name;

    }

private:
    // 存储所有检测到的脸
    std::vector<Rect> faces;
    // 订阅的回调函数
    void subscriber_callback(const sensor_msgs::msg::Image::ConstPtr &msg) // 注意这里的参数类型
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
    void detect_face(cv::Mat &img)
    {
        cv::CascadeClassifier faceDetector;

        // char cwd[1024];
        // getcwd(cwd, sizeof(cwd));
        // std::cout << cwd;  // 打印工作空间路径
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), cwd);

        // 加载特征文件
        faceDetector.load(("./src/base_task4/asset/haarcascade_frontalface_default.xml"));

        if (faceDetector.empty())
        {
            return;
        }

        faceDetector.detectMultiScale(img, faces);

        int count = 1;

        // 遍历所有识别出的脸，画出矩形
        for (auto &face : faces)
        {

            // 绘制标签
            Point origin;
            origin.x = face.x;
            origin.y = face.y - 10; // 稍微偏上一点

            std::string label = "face" + std::to_string(count);
            putText(img, label, origin, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(36, 255, 12), 2);
            // Scalar 是RGB格式的颜色
            rectangle(img, face, Scalar(0, 255, 0), 2);

            count++;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageReciever>();
    // 使用std::bind绑定成员函数和对象实例
    auto callback = std::bind(&ImageReciever::getPosition, node, std::placeholders::_1, std::placeholders::_2);
    rclcpp::Service<custom_interface::srv::ObjectPosition>::SharedPtr service =  node->create_service<custom_interface::srv::ObjectPosition>("get_position", callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}