/*
 * @Author: LHYLHYY 876814061@qq.com
 * @Date: 2024-01-16 14:15:07
 * @LastEditors: LHYLHYY 876814061@qq.com
 * @LastEditTime: 2024-01-16 15:00:44
 * @FilePath: /catkin_ws/src/agv_sim/hfsm_ros/include/hfsm/hardware_init_state.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef HARDWARE_INIT_STATE_HPP
#define HARDWARE_INIT_STATE_HPP

#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <unordered_map>

namespace hfsm_ns
{
    // 硬件初始化状态
    class HardwareInitState : public State
    {
    public:
        std::vector<std::string> hardware_nodes_ = {"/agv_hw_node","/imu_sonser_spec","/laserscan_multi_merger","/mechmind_camera_start",
                                                    "/joy_node"};

    public:
        void start()
        {
            printf_yellow("HardwareInitState start");
            ros::V_string running_node_list;
            ros::master::getNodes(running_node_list);
            int8_t tmp, cnt = 0;
            do
            {
                tmp = 0;
                for (auto target_node : hardware_nodes_)
                    if (std::find(running_node_list.begin(), running_node_list.end(), target_node) != running_node_list.end())
                        continue;
                    else
                    {
                        ROS_ERROR("node %s start failed!", target_node);
                        std::string cmd = "rosnode kill " + target_node;
                        system(cmd.c_str());
                        tmp++;
                    }
                cnt++;
            } while (tmp != 0 && cnt < 10);
            
            if (cnt == 10)
                ROS_ERROR("hardware init error!");
            else
                TransState("Start");
        }

        void stop()
        {
            printf_green("HardwareInitState stop");
        }
    };

}

#endif
