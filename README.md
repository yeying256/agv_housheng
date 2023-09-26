# 舵轮机器人功能包



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


### agv_slam
里面只有几个launch

### 


