# hfsm_ros

## 杨竞博的srv

srv话题 robot_control
0:急停；1：AGV回零；2：夹爪回零；3：两部分一起运动；
4：AGV单独运动；5：夹爪单独运动

srv.request.width 如果是999 那就是不要动

## 视觉相对运动的节点

srv话题名称： /visual_point_move
消息类型： agv_msg::visual_point_move
详细说明：
请求目标相对相机坐标系的期望位姿
geometry_msgs/Pose2D target_pose
反馈的当前位姿
geometry_msgs/Pose2D feed_back_pose
int32 state #1是运行，0是停止 2是查询状态
/---------

bool success #在查询状态中，true是可以给我发指令 false是程序正在运行
string msg #返回状态信息

## 视觉信息话题名称
/DetectorResult_

## 硬件错误日志
|硬件编号   |错误类型     |错误代码   |
|---       |---        |---      |
|0         |PCI初始化失败|0         |
|          |轴使能失败   |1         |
|          |前轮回零失败  |2         |
|          |后轮         |3         |
|          |夹爪         |4         |
