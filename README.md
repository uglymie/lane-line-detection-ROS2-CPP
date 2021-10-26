# lane-line-detection-ROS2-CPP
项目参考优达学城（Udacity）[无人驾驶工程师学位的第二个项目-车道线检测](https://github.com/udacity/CarND-Advanced-Lane-Lines)
将其开发为一个ROS2功能包，ROS2版本为Foxy，开发语言为C++。

# 实现功能

image_pub包的功能为，集读取视频文件、USB相机、RTSP URL为一体，且将图像数据使用sensor_msgs/msg/Image消息发布出来，具体参数可以通过在launch文件里配置。

cpp_advanced_lane_lines包的功能为订阅图像数据，并进行车道线检测流程，最后发布处理后的图像，以及车道线的曲率半径、车辆偏离车道中心点的距离两个数据。

lane_line_msgs功能包为单独创建的消息类型包，用于定义车道线的曲率半径、车辆偏离车道中心点的距离两个数据。

功能包使用C++开发，涉及到优达学城车道线检测流程的openCV + C++实现版本，ROS2的图像数据发布订阅，roslaunch的编写，参数文件的编写和加载，消息文件的编写和加载。

# 一些改进

将原项目中加载棋盘格图像计算相机矩阵以及畸变系数，改为事先计算好，通过launch文件或yaml文件的参数形式加载。

# 编译运行

将功能包复制到自己的工作空间，运行编译命令：
```
colcon build
```
修改launch文件里面视频文件的路径：
```
Node(
    package='image_pub',
    executable='image_pub',
    parameters=[
        {"image_pub_topic_name": '/camera/image_raw'},
        {"capture_format": 'your video path'}
        # {"capture_format": '0'} # USB摄像头
    ]
),
```
这里使用的视频文件可以在最开始提到的优达学城的仓库里下载，其他的资源文件均在编译时复制到share相关目录下，便于加载。
最后刷新. install/setup.bash文件，使用launch文件启动：
```
ros2 launch cpp_advanced_lane_lines lane_line_detection_launch.py
```

# 消息订阅

![image](https://user-images.githubusercontent.com/47886076/138861696-6cc17242-cac4-4c46-b9f6-6028534a9f62.png)

# 图像订阅

![image](https://user-images.githubusercontent.com/47886076/138861727-352befe8-a958-49e0-8681-765c0bd58165.png)

![image](https://user-images.githubusercontent.com/47886076/138861934-fe1e44be-17b1-4bf0-a851-48c678f97964.png)

![image](https://user-images.githubusercontent.com/47886076/138861958-1207a2ea-5861-4b5c-b119-330a4d74f62b.png)
