#include "turn_wheels_controller.h"
#include <pluginlib/class_list_macros.h>

namespace xj_control_ns
{
    turn_wheels_controller::turn_wheels_controller(/* args */)
    {
    }
    
    turn_wheels_controller::~turn_wheels_controller()
    {
    }


    bool turn_wheels_controller::init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh)
    {
        std::vector<std::string> joint_names;
        hardware_interface::VelocityJointInterface* turn_wheel_jointinterface = robot_hw->get<hardware_interface::VelocityJointInterface>();

        int interface_num=turn_wheel_jointinterface->getNames().size();
        
        if(!nh.getParam("joints",joint_names))
        {
            ROS_WARN("joint num is err,or can not read the joint param,joint dof param = %d"
            ,joint_names.size());
            return false;
        }


        if (interface_num==4)
        {
            printf("\033[1;36;40m front %d is robot joint \033[0m \n",interface_num);
        }
        return true;
    }


    void turn_wheels_controller::starting(const ros::Time& time)
    {
        ;
    }
    void turn_wheels_controller::update(const ros::Time& time, const ros::Duration& period)
    {
        ;
    }
}

PLUGINLIB_EXPORT_CLASS(xj_control_ns::turn_wheels_controller,
                       controller_interface::ControllerBase)
