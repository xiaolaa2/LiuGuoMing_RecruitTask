#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_publisher", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));

        std::string rtsp1 = "rtsp://172.20.192.1/live";
        cv::VideoCapture cap(rtsp1, CAP_FFMPEG); // 打开默认摄像头
        // cv::VideoCapture cap("./src/base_task4/asset/face.mp4");

        if (!cap.isOpened())
        {
            std::cout << "Error opening camera" << std::endl;
            return;
        }

        cv::Mat frame;

        while (true)
        {

            cap >> frame; // 获取一帧

            // cv::imshow("Camera", frame); // 显示

            auto message = sensor_msgs::msg::Image();

            // 使用cv_bridge将视频每一帧转换为ros2的sensor信息, 同时设置Encoding的参数为bgr8
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",frame).toImageMsg(message);
            publisher_->publish(message);

            char c = (char)cv::waitKey(25); // 每25毫秒读取一次键盘输入
            if (c == 27)
            { // ESC键退出
                break;
            }
        }

        cap.release();
        cv::destroyAllWindows();
    
    }

    std::string currentDateTime()
    {

        // 获取当前时间点
        auto now = std::chrono::system_clock::now();

        // 把时间点转换为时间戳
        auto timestamp = std::chrono::system_clock::to_time_t(now);

        // 把时间戳转换为本地时间
        struct tm *local_time = localtime(&timestamp);

        // 定义格式化的格式
        char formatted[100];

        // 格式化本地时间
        strftime(formatted, sizeof(formatted), "%Y-%m-%d %H:%M:%S", local_time);

        return std::string(formatted);
    }

private:
    // 发布的回调函数
    void timer_callback()
    {

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
