#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace xj_control_ns
{
    class Agv_hw_interface: public hardware_interface::RobotHW
    {
    private:
        /* data */

        std::vector<std::string> agv_joint_name;

        std::vector<double> joint_position_state;
        std::vector<double> joint_velocity_state;
        std::vector<double> joint_effort_state;
        std::vector<double> joint_velocity_command;
        int agv_num_joints_;


        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::VelocityJointInterface velocity_joint_interface;
    public:
        Agv_hw_interface(/* args */);
        ~Agv_hw_interface();


        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
        void read(const ros::Time& time, const ros::Duration& period);
        void write(const ros::Time& time, const ros::Duration& period);

    };
    

    
}
