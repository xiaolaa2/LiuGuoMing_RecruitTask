# 实现过程
1. 使用launch文件来方便程序的启动
2. 播放视频。通过ros2 bag play 命令来将视频以topic的格式发布到/color/image_raw上
3.  ArmorDetectNode节点，通过subscriber将视频topic订阅下来并通过，cv_bridge::toCvShare将视频的sensor_msg信息转换为cv::Mat后进行一系列装甲板检测操作

## 装甲板检测 
### 图片预处理
1. 转换为灰色图像
2. 高斯滤波：使用GaussianBlur将视频内的光照噪声去除
3. 二值化：使用threshold对高斯滤波后的图像进行图像二值化

### 灯条检测
1. 新建一个Light类，其继承于cv::RotatedRect，添加了width，height，color变量来记录灯条的长宽和颜色
2. 使用opencv的findContours函数找到所有亮点的轮廓，然后使用minAreaRect找到每个轮廓的最小包围矩形，使用isLight函数来检测该轮廓是否是灯条，使用detectLightColor函数来检测灯条颜色

**isLight**
根据灯条的`宽高比率`、`倾斜角度`来筛选

**detectLightColor**
1. 首先先根据灯条的包围矩形获得roi区域
2. 将roi区域转换到hsv色彩空间
3. 根据颜色阈值使用inRange函数分别提取出蓝色和红色的区域
4. 使用countNonZero函数获得蓝色和红色的像素总数，最后比较两种颜色的像素总数，则该灯条的颜色就是像素数多的那一种颜色

**宽高计算方法**
1. 高度：通过矩形上边的中点和下边的中点组成的向量，测量其长度可以得到
2. 宽度：通过矩形下面两个点可以获得宽度

### 装甲板检测
1. 新建Armor类，其内容与Light类相同，但多了一个Point center 来存储装甲板的中点
2. 先对所有Light按照中点的x坐标从左往右排序，遍历找到的所有Light，两两进行配对，如果两个灯条颜色相同，则配对成功进入下一步
3. 根据两个灯条的八个端点组成的轮廓计算出一个最小包围矩形，再通过isArmor函数判断这个矩形是否是装甲板
4.若是则进行装甲板数字的识别

**数字识别**
1. 使用opencv进行透视变换，消除数字的角度变化
2.完成了数字的图像提取，但是还没有完成识别，因为数据集问题所以我训练的模型识别精度很差，想要使用别人开源的模型，但是不知道为什么用不了rm——vision提供的onnx模型