#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <iostream>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "agv_hw.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "hw_interface_node");
    

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;
    xj_control_ns::Agv_hw_interface hw;
    int ret = hw.init(nh,nh);

    ROS_INFO("hw init ret=%d",ret);
    

    controller_manager::ControllerManager cm(&hw,nh);
    sleep(1);


    
    ros::Duration period(1.0/1000);
    ROS_INFO("joint hw run");

    while (ros::ok())
    {
        hw.read(ros::Time::now(),period);
        cm.update(ros::Time::now(),period);
        hw.write(ros::Time::now(),period);
        period.sleep();
    }
    spinner.stop();
    return 0;
}
