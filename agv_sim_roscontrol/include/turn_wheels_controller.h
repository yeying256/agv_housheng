#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>

#include <realtime_tools/realtime_buffer.h>


#include <hardware_interface/hardware_interface.h>
#include <controller_interface/multi_interface_controller.h>

#include "robot_dynamic.h"




namespace xj_control_ns
{
    class turn_wheels_controller:
    public controller_interface::MultiInterfaceController
    <hardware_interface::VelocityJointInterface> 
    {
    private:
        /* data */
        int wheel_num;
    public:
        turn_wheels_controller(/* args */);
        ~turn_wheels_controller();
        bool init(hardware_interface::RobotHW* robot_hw,ros::NodeHandle& nh);
        void starting(const ros::Time& /*time*/);
        void update(const ros::Time& time, const ros::Duration& period);
    };
    

    

}