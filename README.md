# 舵轮机器人功能包

## 获取完整功能包
如果首次克隆仓库及其模块，使用：
```
git clone --recursive 仓库地址
```

对于仓库首次拉取模块，可以使用:
```
git submodule update --init --recursive
```

更新子模块:
```
git submodule update --recursive --remote
```

## 使用
在新环境下，请安装依赖的功能包

第一次不管不顾直接运行，一定会出现gazebo卡死的现象，将路径
```
agv_sim/agv_urdf_test/src/agv_sim/agv_urdf_test/world
```
下的$ar_targ$文件夹整个复制到
```
~/.gazebo/models/
```
下。
另外，将下列链接的模型下载下来
```
https://github.com/osrf/gazebo_models
```
将下载下来的模型放置到：
```
~/.gazebo/models/
```
文件夹下。
教程链接：
```
https://blog.csdn.net/weixin_45498383/article/details/128938766
```

可能需要提前编译一些自定义文件
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="agv_msg"
```

```
catkin_make -DCATKIN_WHITELIST_PACKAGES="ar_track_alvar_msgs"
```
然后运行
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

```
改回来，要不然以后只是编译这两个功能包

## 依赖的功能包
需要将这些功能包编译然后加载到动态链接库中

### serial
串口包

```
sudo apt-get install ros-noetic-serial
```

### robot_dynamic_cal
作用：用到了舵轮运动学解算
链接：
```
https://jihulab.com/xjtu_robot_wx/ros/robot_dynamic_cal
```

### MatLogger2
作用：记录matlab可直接读取的数据
链接：
```
https://xj-robot.yuque.com/houhyu/xypu3s/xswpi3a6ldfhc9gy
```
官网链接：
```
https://github.com/ADVRHumanoids/MatLogger2

```
### ros-control
提示 找不到#include <combined_robot_hw/combined_robot_hw.h>
```
sudo apt-get install ros-noetic-ros-control
```

### pcl_conversions
```
sudo apt-get install ros-noetic-pcl-conversions 
```

### pcl_ros

```
sudo apt-get install ros-noetic-pcl-ros

```

### bfi
```
https://blog.csdn.net/qq_39779233/article/details/128406859
```

### amcl
```
sudo apt-get install ros-noetic-amcl
```

### map_server

```
sudo apt-get install ros-noetic-map-server
```

### move_base

```
sudo apt-get install ros-noetic-move-base

```

## 使用

### 仿真环境启动
```
roslaunch agv_run_all agv_sim_all.launch
```



## 功能包介绍

### agv_sim_robot_hw
这里是robothw节点启动的地方，里面有一个进程，用来连接硬件抽象层robot_hw和ros_control

### agv_sim_roscontrol
ros_control可以连接robot_hw以及gazebo的仿真环境

带遥控器和gazebo的launch,可以使用遥控器对机器人进行控制

```
roslaunch agv_sim_roscontrol agv_sim_roscontrol_joy.launch
```


### agv_slam
里面只有几个launch
可以进行建图
```
roslaunch agv_slam agv_slam.launch
```
### ira_laser_tools
激光雷达融合的包

### robot_pose_ekf
里程计imu，视觉里程计融合的包，用的卡尔曼滤波算法
用到了 $$ orocos-bfl$$ 包,可以安装一下
安装过程：cd到那个包的路径
```
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

### agvsim_navigation
导航规划功能包，包括全局规划A*和局部规划TEB
启动导航终端命令：
```
roslaunch agvsim_navigation move_base.launch
```
### agv_relative_move
根据状态机给出的信息，


# 目前问题

1雷达安装不牢固导致定位不准，容易扫描到地面误识别为障碍物，同时安装位置不准，导致雷达数据肉眼可见的歪，稍作补偿效果稍好些。
2轮子精度不高，响应慢，导致运行过程中小概率会卡死，使编码器丢步，同时导致里程计不准，进而影响全局定位精度。
3视觉算法精度不足以稳定完成任务，机器人不动，在同一个位置拍出来的照片数据是相同的，算出来的结果差距不大，数据波动只有1-2个mm，但这只能说明相机数据足够准。换一个位置识别出来的相对位置就会不一样，但到底不一样到什么程度很难测量，因为定位和里程计都不准，用一个不准的数据去评估另一个不准的数据，没什么意义。想提高视觉检测精度，只能换算法。
4因为视觉，定位，轮子执行都不准，整个机器人全方位的精度差，所以无法互相验证，也无法采集采集数据以证明任何一个数据不准。只能看现象。
5回零还是不行，有一定概率回零失败
6经常识别不到
7解决了视觉信息错位问题，但导致拍照过程中容易进入死循环，检测不到目标跳不出来，切了状态还是一直拍照。

# 


