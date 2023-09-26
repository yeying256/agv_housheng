#include "agv_hw.h"


namespace xj_control_ns
{
    Agv_hw_interface::Agv_hw_interface(/* args */)
    {
    }
    
    Agv_hw_interface::~Agv_hw_interface()
    {
    }

    bool Agv_hw_interface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
    {
        bool ret = robot_hw_nh.getParam("/agv_sim/robot_hw_test/joints", agv_joint_name);//从参数服务器中获取name
        if (ret == true)
        {
            std::cout<<"\033[1;32;40m 获得jointname成功\033[0m"<<std::endl;
        }else
        {
            std::cout<<"\033[1;31;40m 获得jointname失败\033[0m"<<std::endl;
        }
        
        ROS_ERROR("getParam ret= %d",ret);
        
        agv_num_joints_ = agv_joint_name.size();
        for (size_t i = 0; i < agv_num_joints_; i++)
        {
            ROS_ERROR("jointname=%s",agv_joint_name[i].c_str());
        }
        joint_effort_state.resize(agv_num_joints_);
        joint_position_state.resize(agv_num_joints_);
        joint_velocity_state.resize(agv_num_joints_);

        joint_velocity_command.resize(agv_num_joints_);


        for(int i=0; i<agv_num_joints_; i++)
        {
            //State
            hardware_interface::JointStateHandle jointStateHandle(agv_joint_name[i].c_str(), &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
            joint_state_interface.registerHandle(jointStateHandle);
            ROS_INFO("joint_name[%d].c_str()=%s",i,jointStateHandle.getName().c_str());

            //Velocity
            hardware_interface::JointHandle jointVelocityHandle(joint_state_interface.getHandle(agv_joint_name[i].c_str()), &joint_velocity_command[i]);
            velocity_joint_interface.registerHandle(jointVelocityHandle);
        }

        registerInterface(&joint_state_interface);          //将类中的接口注册到ros中

        registerInterface(&velocity_joint_interface);    




        return true;
    }

    void Agv_hw_interface::read(const ros::Time& time, const ros::Duration& period)
    {

        ;
    }
    void Agv_hw_interface::write(const ros::Time& time, const ros::Duration& period)
    {

        for(int i=0;i < agv_num_joints_;i++)
        {
         ROS_WARN("recive from controller joint %d is %f",i,joint_velocity_command[i]);
        
        }

        ;
    }

}
PLUGINLIB_EXPORT_CLASS(xj_control_ns::Agv_hw_interface, hardware_interface::RobotHW)