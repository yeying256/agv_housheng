#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*
 move_base_msgs::MoveBaseAction
 move_base在world中的目标
*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavState
{
private:
    MoveBaseClient ac;
    move_base_msgs::MoveBaseGoal goal; // 目标的属性设置
public:
    // 创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
    NavState() : ac("move_base", true){};
    int set_goal(float set_x, float set_y, float set_orientation);
};

int NavState::set_goal(float set_x, float set_y, float set_orientation)
{
    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");
    // Send a goal to move_base
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = set_x;
    goal.target_pose.pose.position.y = set_y;
    goal.target_pose.pose.orientation.w = set_orientation;
    ROS_INFO("goal is: %f, %f, %f", set_x, set_y, set_orientation);
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    // Wait for the action to return
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("You have reached the goal!");
        return 1;
    }
    else
    {
        ROS_INFO("The base failed for some reason");
        return 0;
    }
}
