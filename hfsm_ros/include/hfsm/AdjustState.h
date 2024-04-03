#ifndef ADJUSTSTATE_H
#define ADJUSTSTATE_H
#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <agv_msg/Button.h>
#include <agv_msg/reltive_pose.h>
#include <unordered_map>
#include <agv_msg/grab_agv.h>
#include <agv_msg/reltive_pose_visual.h>
#include <agv_msg/visual_point_move.h>



namespace hfsm_ns
{
    // 自动状态-调整状态
class AdjustState : public State
{
private:


public:

// WIDTH宽度....HEIGHT高度
    enum Data_type{
        WIDTH, //宽度
        HEIGHT //高度
    };
    ros::NodeHandle n;
    ros::ServiceClient visual_client;

    agv_msg::visual_point_move::Request req;
    agv_msg::visual_point_move::Response resp;
    std::unordered_map<int8_t, workbench> workbench_map;

public:
    AdjustState(ros::NodeHandle nh);
    void start();

    EventDeal DealEvent(EventData &event_data);

    void stop();

    /**
     * @brief 
     * 
     * @param RealData 真实的数据单位 mm
     * @param Types 要转换的数据类型  WIDTH, //宽度 HEIGHT //高度
     * @return double 转换出来的数据
     */
    static double RealData2SrvData(double RealData, Data_type Types);


    /**
     * @brief 调用srv通讯向robot_hw发送夹爪开合和上升的指令
     * 
     * @param width 真实的宽度（外侧）
     * @param hight 真实的高度（前部尖端的高度）
     * @param width_move 是否宽度运动
     * @return true 
     * @return false 
     */
    bool grab_move(double width,double hight,bool width_move);

    
    /**
     * @brief 切换agv模式 0:急停；1：AGV回零；2：夹爪回零；3：两部分一起运动；4：AGV单独运动；5：夹爪单独运动
     * 
     * @param mode 0:急停；1：AGV回零；2：夹爪回零；3：两部分一起运动；4：AGV单独运动；5：夹爪单独运动
     * @return true 
     * @return false 
     */
    bool Switch_agv_mode(int mode);


    /**
     * @brief 相对运动指令
     * 
     * @param x_d 期望x，在相机坐标系下料桶的x，期望位置
     * @param y_d 同x
     * @param yaw_d 同x
     * @param feedback 相机反馈回来的当前物料
     * @return true 调用成功
     * @return false 调用失败
     */
    bool relative_move_cmd(double x_d,double y_d,double yaw_d,geometry_msgs::Pose2D feedback);


    /**
     * @brief 相对运动指令
     * 
     * @param mode 模式 0停止 2查询状态 1：给出指令，不能在这调用
     * @return true 
     * @return false 
     */
    bool relative_move_cmd(int mode);



    /**
     * @brief 
     * 
     * @param pre_height 预备状态夹爪的高度
     * @param pre_width 预备状态夹爪宽度
     * @param grab_width 动作的时候夹爪的宽度
     * @param grab_height 动作时候夹爪的高度
     * @param workbench_id 
     */
    void run(int pre_height, int pre_width, int grab_width, int grab_height, int workbench_id);
    

    void update();


};
}


#endif