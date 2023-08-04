# 问题
1. wsl2无法使用windows的摄像头
2. wsl2如何访问主机服务
3. CascadeClassifier的使用要导入opencv2/objdetect.hpp包

# 解决方法
1. 使用obs以rtsp协议进行推流，然后在wsl2内使用opencv获取rtsp流
2. 根据wsl2文档https://learn.microsoft.com/zh-cn/windows/wsl/networking，先用cat /etc/resolv.conf找到windows主机的地址就可以了
