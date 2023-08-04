# 问题
1. wsl2无法使用windows的摄像头
2. wsl2如何访问主机服务
3. CascadeClassifier的使用要导入opencv2/objdetect.hpp包

# 解决方法
1. 使用obs以rtsp协议进行推流，然后在wsl2内使用opencv获取rtsp流
2. 根据wsl2文档https://learn.microsoft.com/zh-cn/windows/wsl/networking，先用cat /etc/resolv.conf找到windows主机的地址就可以了

# 注意
为了方便这里我使用了opencv自带的级联分类器和人脸特征文件来检测人脸的存在，同时使用本地录制好的文件进行测试，不然太卡。

# 实现过程
1. 新建两个节点，分别作为订阅和发布者
2. 发布者：通过opencv打开摄像头并实时获取画面，使用cv_bridge将Opencv的Mat转为ros2里的sensor_msgs，并通过publisher将信息发布出去
3. 订阅者：订阅topic里的sensor_msgs信息，使用cv_bridge将sensor_msgs转换为opencv的mat，之后将图片进行物品的识别。