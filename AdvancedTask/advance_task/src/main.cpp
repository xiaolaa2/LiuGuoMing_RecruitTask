#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;
using namespace cv;
using namespace std;

enum class LightColor
{
    RED,
    BLUE
};

class Armor : public cv::RotatedRect
{
public:
    Armor(cv::RotatedRect rect) : cv::RotatedRect(rect)
    {
        // 先获得旋转矩形的四个顶点
        cv::Point2f p[4];
        rect.points(p);
        // 按照y坐标排序四个顶点,p[0]和p[1]是下面两个顶点
        sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
             { return a.y < b.y; });
        // 取两个中点进行计算
        auto bottom = (p[0] + p[1]) / 2;
        auto top = (p[2] + p[3]) / 2;
        center = (top + bottom) / 2;

        // 获取两个向量来获取向量的长度
        height = norm(top - bottom);
        width = norm(p[0] - p[1]);

        auto vec = top - bottom;
        // 将弧度转换我角度
        auto angle = atan2(vec.y, vec.x) * 180 / CV_PI;

        rotate_angle = angle;
    }
    float height;
    float width;
    float rotate_angle;
    int number;
    Point center;
    LightColor color;
};

// 灯条类
class Light : public cv::RotatedRect
{
public:
    // 无参构造函数
    Light(cv::RotatedRect rect) : cv::RotatedRect(rect)
    {
        // 先获得旋转矩形的四个顶点
        cv::Point2f p[4];
        rect.points(p);
        // 按照y坐标排序四个顶点,p[0]和p[1]是下面两个顶点
        sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
             { return a.y < b.y; });
        // 取两个中点进行计算
        auto bottom = (p[0] + p[1]) / 2;
        auto top = (p[2] + p[3]) / 2;

        // 获取两个向量来获取向量的长度
        height = norm(top - bottom);
        width = norm(p[0] - p[1]);

        auto vec = top - bottom;
        // 将弧度转换我角度
        auto angle = atan2(vec.y, vec.x) * 180 / CV_PI;

        rotate_angle = angle;
    }

    float height;
    float width;
    float rotate_angle;
    LightColor color;
    std::vector<cv::Point> contour; // 该灯条的轮廓
private:
};

class ArmorDetectNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // 预处理后图片发布
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preprocess_img_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr number_roi_publisher_;

    cv::Mat preProcess(const cv::Mat &img)
    {
        // 先灰度化再二值化
        cv::Mat gray;
        cv::Mat binary;
        // 高斯滤波,去除噪声影响
        Mat gaus;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
        GaussianBlur(gray, gaus, cv::Size(15, 15), 0);
        cv::threshold(gaus, binary, 127, 255, cv::THRESH_BINARY);

        // imshow("pre", binary);

        return binary;
    }

    // 判断是否是装甲板
    bool isArmor(Armor &armor)
    {
        return true;
    }

    // 判断是否是灯条的函数
    bool isLight(Light &light)
    {

        // 灯条比率
        float ratio = light.width / light.height;
        float max_ratio = 0.5;
        float min_ratio = 0.32;
        float min_angle = 80;
        float max_angle = 120;

        if (ratio > max_ratio || ratio < min_ratio)
        {
            return false;
        }

        // 限制倾斜角度
        if (light.rotate_angle < min_angle || light.rotate_angle > max_angle)
        {
            return false;
        }

        return true;
    }

    // 检测灯条颜色
    void detectLightColor(Light &light, const cv::Mat &img)
    {
        // 获得灯条的局部roi区域
        cv::Mat roi = img(light.boundingRect());
        // imshow("roi", roi);
        cv::Mat hsv; // hsv颜色空间

        // 颜色阈值
        cv::Scalar lower_red(0, 50, 50);
        cv::Scalar upper_red(3, 255, 255);
        cv::Scalar lower_blue(100, 50, 50);
        cv::Scalar upper_blue(124, 255, 255);

        cvtColor(roi, hsv, COLOR_BGR2HSV);

        // 使用inRange提取蓝色和红色区域
        cv::Mat mask_red, mask_blue;
        cv::inRange(hsv, lower_red, upper_red, mask_red);
        cv::inRange(hsv, lower_blue, upper_blue, mask_blue);

        // imshow("red_mask", mask_red);
        // imshow("blue_mask", mask_blue);

        // 统计红色或者蓝色的像素总数
        int red_cnt = cv::countNonZero(mask_red);
        int blue_cnt = cv::countNonZero(mask_blue);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "original point is (%d, %d)", red_cnt, blue_cnt );
        if (red_cnt > blue_cnt)
        {
            // 红色像素比较多那么就是红色的灯条
            light.color = LightColor::RED;
        }
        else
        {
            light.color = LightColor::BLUE;
        }
    }

    // 装甲板数字检测
    void numberDetect(Armor &armor, const cv::Mat &img) {
        cv::Mat roi = img(armor.boundingRect());
        cv::Mat number_image;

        // 二值化获得数字图像
        cv::cvtColor(roi, number_image, cv::COLOR_RGB2GRAY);
        GaussianBlur(number_image, number_image, cv::Size(5, 5), 0);
        cv::threshold(number_image, number_image, 100, 130, cv::THRESH_BINARY);

        Mat mask = number_image == 0; // 反转mask,选择非黑色像素
        bitwise_not(mask, mask); // 将遮罩取反，求出二值化后图像不是黑色的区域
        number_image.setTo(Scalar(255), mask); // 将mask中的像素设置为255白色
        // bitwise_not(number_image, number_image); // 将白色的字变为黑色

        auto message = sensor_msgs::msg::Image();
        // 使用cv_bridge将视频每一帧转换为ros2的sensor信息, 同时设置Encoding的参数为bgr8
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", number_image).toImageMsg(message);
        number_roi_publisher_->publish(message);
    }

    // 装甲板检测
    void ArmorDetect(const cv::Mat &img)
    {
        cv::Mat raw_img;
        img.copyTo(raw_img);
        cv::Mat preProcessImg = preProcess(img);
        vector<Light> lights;
        vector<Armor> armors;

        // 先找到轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(preProcessImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 检测灯条
        for (auto &contour : contours)
        {
            // 获得最小的包围矩形,注意这个矩形是可以倾斜的
            auto rect = minAreaRect(contour);
            auto light = Light(rect);

            // 通过一系列限制条件来控制是否是灯条
            if (isLight(light))
            {
                // 检测灯条颜色
                detectLightColor(light, raw_img);
                light.contour = contour; // 填充灯条的轮廓以便后面获得装甲板
                lights.emplace_back(light);
                // boundingRect获取最小包围的矩形，这个矩形是水平的
                rectangle(img, light.boundingRect(), Scalar(0, 255, 0), 2);

                // 设置标签字体
                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double fontScale = 0.5;

                // 设置标签颜色和线条宽度
                int thickness = 1;
                cv::Scalar color(0, 255, 0); // BRG格式

                // 计算文字大小,获取 baseline
                int baseline;
                cv::Size textSize = cv::getTextSize("red", fontFace, fontScale, thickness, &baseline);

                // 计算文字左下角坐标
                cv::Point origin(light.boundingRect().x, light.boundingRect().y + textSize.height);
                if (light.color == LightColor::RED)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "red");
                    cv::putText(img, "red", origin, fontFace, fontScale, color, thickness);
                }
                else if (light.color == LightColor::BLUE)
                {
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "blue");
                    cv::putText(img, "blue", origin, fontFace, fontScale, color, thickness);
                }
            }
        }

        vector<Point> contour_bak; // 存储两个灯条的轮廓
        int count = 0;
        // 将所有灯条按x坐标从左到右排好序
        sort(lights.begin(), lights.end(), [](const Light &a, const Light &b)
        { return a.center.x < b.center.x; });
        // 检测装甲板,两两遍历灯条
        for (size_t i = 0; i < lights.size(); i++)
        {
            for (size_t k = i + 1; k < lights.size(); k++)
            {
                // 先判断两个灯条的颜色是否相同
                if (lights[i].color == lights[k].color)
                {
                    // 先获得两个灯条旋转矩形的四个顶点
                    cv::Point2f p1[4];
                    lights[i].points(p1);
                    cv::Point2f p2[4];
                    lights[k].points(p2);
                    // 按照x坐标排序四个顶点取左边灯条的右边两个点，取右边灯条的左边两个点
                    sort(p1, p1 + 4, [](const cv::Point2f &a, const cv::Point2f &b)
                        { return a.x < b.x; });
                    sort(p2, p2 + 4, [](const cv::Point2f &a, const cv::Point2f &b)
                    { return a.x < b.x; });
                    contour_bak.clear();
                    // contour_bak = lights[i].contour;
                    // contour_bak.insert(contour_bak.end(), lights[k].contour.begin(), lights[k].contour.end());
                    contour_bak.emplace_back(p1[2]);
                    contour_bak.emplace_back(p1[3]);
                    contour_bak.emplace_back(p2[0]);
                    contour_bak.emplace_back(p2[1]);
                    auto armor = Armor(minAreaRect(contour_bak));
                    armor.color = lights[i].color;

                    if (isArmor(armor))
                    {
                        // 设置标签字体
                        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                        double fontScale = 0.5;

                        // 设置标签颜色和线条宽度
                        int thickness = 1;
                        cv::Scalar color(0, 0, 255); // BRG格式

                        // 计算文字大小,获取 baseline
                        int baseline;
                        cv::Size textSize = cv::getTextSize("red", fontFace, fontScale, thickness, &baseline);

                        // 计算文字左下角坐标
                        cv::Point origin(armor.boundingRect().x, armor.boundingRect().y + textSize.height);
                        cv::putText(img, string("Armor") + to_string(++count), origin, fontFace, fontScale, color, thickness);
                        rectangle(img, armor.boundingRect(), Scalar(0, 0, 255), 2);
                        numberDetect(armor, raw_img);
                        armors.emplace_back(armor);
                    }
                }
            }
        }

        // imshow("after", img);

        auto message = sensor_msgs::msg::Image();
        // 使用cv_bridge将视频每一帧转换为ros2的sensor信息, 同时设置Encoding的参数为bgr8
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg(message);
        preprocess_img_publisher_->publish(message);
    }

    // 从topic中获得视频信息
    void getImage(const sensor_msgs::msg::Image::ConstPtr &msg)
    {
        // 使用cvbridge转换信息为图片
        cv::Mat frame;
        cv_bridge::CvImageConstPtr cv_ptr; // 获取cv图像的引用指针
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        frame = cv_ptr->image;

        // cv::imshow("screen", frame);

        // 对获得的每帧进行处理
        ArmorDetect(frame);

        cv::waitKey(2);
    }

public:
    ArmorDetectNode() : Node("armor_detect_node")
    {
        // 创建图像订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/color/image_raw", 10, std::bind(&ArmorDetectNode::getImage, this, _1));
        preprocess_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/color/preprocess_img", 10);
        number_roi_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/color/number_roi", 10);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectNode>());
    rclcpp::shutdown();
    return 0;
}