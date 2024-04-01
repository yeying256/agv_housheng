/*
 * @Author: LHYLHYY 876814061@qq.com
 * @Date: 2024-01-30 18:12:50
 * @LastEditors: LHYLHYY 876814061@qq.com
 * @LastEditTime: 2024-03-31 20:36:38
 * @FilePath: /catkin_ws/src/agv_sim/hfsm_ros/include/hfsm/SendGoal.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SEND_GOAL_H__
#define __SEND_GOAL_H__

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "agv_msg/GP_nav.h"
/*
 move_base_msgs::MoveBaseAction
 move_base在world中的目标
*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavState
{
private:
    move_base_msgs::MoveBaseGoal goal; // 目标的属性设置
    agv_msg::GP_nav gp_nav; //目标设置


public:
#ifdef USE_MOVE_BASE
    MoveBaseClient ac;
#endif
    ros::ServiceClient GP_srv_;
    ros::NodeHandle nh_;

public:
    // 创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
    NavState()
    #ifdef USE_MOVE_BASE
     : ac("move_base", true)
    #endif
    {

        // visual_client = n.serviceClient<agv_msg::visual_point_move>("/visual_point_move");

        GP_srv_ = nh_.serviceClient<agv_msg::GP_nav>("/GP_nav");
    }
    int set_goal(float set_x, float set_y, geometry_msgs::Quaternion set_orientation)
    {
        // Wait 60 seconds for the action server to become available

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = set_x;
        goal.target_pose.pose.position.y = set_y;
        goal.target_pose.pose.orientation = set_orientation;
        #ifdef USE_MOVE_BASE
        ROS_INFO("Waiting for the move_base action server");
        ac.waitForServer(ros::Duration(60));
        ROS_INFO("Connected to move base server");

        ROS_INFO("goal is: %f, %f", set_x, set_y);
        ROS_INFO("Sending goal");
        // 用movebase的发送代码
        ac.sendGoal(goal);
        

        #else
        gp_nav.request.pose.pose=goal.target_pose.pose;
        //0即停 1指令 2查询
        gp_nav.request.state = 1;
        GP_srv_.call(gp_nav);

        #endif
        

        return 0;
    }
};

#endif