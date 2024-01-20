#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include "zmotion.h"
#include "agv_msg/grab_agv.h"
#include "agv_msg/hw_srv.h"

#include "zmotion.h"
#include "zmcaux.h"

#define GRAB_OPEN 0
#define GRAB_RIGHT 1
#define GRAB_LEFT 2
#define FRONT_STEER 3
#define FRONT_WHEEL 4
#define BACK_STEER 5
#define BACK_WHEEL 6

namespace xj_control_ns
{
    class Agv_hw_interface : public hardware_interface::RobotHW
    {
    private:
        std::vector<std::string> agv_joint_name;
        std::vector<double> joint_position_state;
        std::vector<double> joint_velocity_state;
        std::vector<double> joint_effort_state;
        std::vector<double> joint_velocity_command;
        int agv_num_joints_;
        int status_;

        ros::ServiceClient client;

        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::VelocityJointInterface velocity_joint_interface;

        ros::ServiceServer command_service_;
        ros::ServiceClient If_zero_;
        ros::NodeHandle nh;
        ros::Publisher log_pub_;

    public:
        Agv_hw_interface(/* args */);
        ~Agv_hw_interface();

        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);

        bool Grab_Server(agv_msg::grab_agv::Request &req,
                         agv_msg::grab_agv::Response &resp);
        void commandCheckHandler(const char *command, int ret);
        ZMC_HANDLE handle; // 连接句柄
    };

}