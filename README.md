# 舵轮机器人功能包


## 使用
在新环境下，请安装依赖的功能包
将路径
```
agv_sim/agv_urdf_test/src/agv_sim/agv_urdf_test/world
```
下的$ar_targ$文件夹整个复制到
```
~/.gazebo/models/
```
下


## 依赖的功能包
需要将这些功能包编译然后加载到动态链接库中

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
### pcl_conversions
```
sudo apt-get install ros-noetic-pcl-conversions 
```

### pcl_ros

```
sudo apt-get install ros-noetic-pcl-ros

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

可以直接运行

```
roslaunch agvsim_navigation navigation_gazebo.launch
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



