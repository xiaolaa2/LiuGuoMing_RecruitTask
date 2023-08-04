# 遇到的问题
1. 无法使用相对路径读取图片
使用了getcwd()函数之后发现工作空间是在/home/xiaolaa/ros2_workspace/src里，所以图片的路径应该是./base_task3(包名)/asset/test_image.jpg

# 实现过程
1. 首先先使用cvtColor函数来转为灰度图
2. 使用高斯滤波将散光等噪声点去掉
3. 使用Canny边缘检测来获得物品的轮廓