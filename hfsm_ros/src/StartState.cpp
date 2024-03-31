/*
 * @Author: LHYLHYY 876814061@qq.com
 * @Date: 2023-12-02 22:39:46
 * @LastEditors: LHYLHYY 876814061@qq.com
 * @LastEditTime: 2024-03-31 12:27:08
 * @FilePath: /catkin_ws/src/agv_sim/hfsm_ros/src/StartState.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "hfsm/StartState.h"

namespace hfsm_ns
{
    void StartState::start()
    {
        printf_yellow("StartState start");
        this->_context->client.waitForExistence();
        // AGV回零
        this->_context->srv.request.status = 1;
        this->_context->client.call(this->_context->srv);
        while (ros::ok())
        {
            if (this->_context->srv.response.agv_result == 0)
            {
                ROS_INFO("STEERING_WHEEL ready");
                break;
            }
            else
            {
                ROS_ERROR("STEERING_WHEEL reset failed!");
                system("rosnode kill /agv_hw_node");
            }
        }

        sleep(1);

        // 夹爪回零
        this->_context->srv.request.status = 2;
        this->_context->client.call(this->_context->srv);
        while (ros::ok())
        {
            if (this->_context->srv.response.grab_result == 1)
            {
                ROS_INFO("GRAB ready");
                break;
            }
            else
            {
                ROS_ERROR("GRAB reset failed!");
                system("rosnode kill /agv_hw_node");
            }
        }

        // 启动导航和视觉
        system("gnome-terminal -e \'roslaunch agvsim_navigation nav_vision.launch\'");

        TransState("Idle");
    }
    void StartState::stop()
    {
        printf_green("StartState stop");
    }

    void StartState::update()
    {
        ROS_INFO("Starting...");
    }

}