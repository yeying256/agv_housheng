/*
 * @Author: LHYLHYY 876814061@qq.com
 * @Date: 2024-01-30 18:12:50
 * @LastEditors: LHYLHYY 876814061@qq.com
 * @LastEditTime: 2024-04-10 18:19:39
 * @FilePath: /catkin_ws/src/agv_sim/hfsm_ros/include/hfsm/idle_state.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef IDLE_STATE_HPP
#define IDLE_STATE_HPP

#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <unordered_map>
#include "mecheye_ros_interface/CaptureColorMap.h"
#include "mecheye_ros_interface/CaptureDepthMap.h"
#include "agv_msg/detector.h"

namespace hfsm_ns
{
    // 硬件初始化状态
    class IdleState : public State
    {
    public:
        IdleState()
        {
            client = n_.serviceClient<agv_msg::detector>("/detector");

            color_map_client_ = n_.serviceClient<mecheye_ros_interface::CaptureColorMap>("/capture_color_map");
            depth_map_client_ = n_.serviceClient<mecheye_ros_interface::CaptureDepthMap>("/capture_depth_map");
        }

        void start()
        {
            printf_yellow("IdleState start");
            _context->detector_type = DetectorType((int)detecting_);
            agv_msg::GP_nav gp_nav;
            gp_nav.request.state = 0;
            _context->nav.GP_srv_.call(gp_nav);
            bool ret = ros::service::waitForService("capture_color_map", 10);
            if (!ret)
            {
                ROS_ERROR("ColorMap capture service start failed");
                return;
            }
            ret = ros::service::waitForService("capture_depth_map", 10);
            if (!ret)
                ROS_ERROR("DepthMap capture service start failed");
            camera_state_ = true;
            std::function<EventDeal(EventData &)> func = std::bind(&IdleState::DealEvent, this, std::placeholders::_1);
            set_event_func(func);
        }

        void stop()
        {
            printf_green("IdleState stop");
            _context->detector_type = DetectorType((int)sleep_);
        }

        EventDeal DealEvent(EventData &event_data)
        {
            if ((EventS)event_data._event_type == photo_)
            {
                if (camera_state_ == true)
                {
                    mecheye_ros_interface::CaptureDepthMap depth_srv;
                    depth_map_client_.call(depth_srv);
                    mecheye_ros_interface::CaptureColorMap color_srv;
                    color_map_client_.call(color_srv);
                    sleep(0.1);
                    agv_msg::detector::Request req;
                    agv_msg::detector::Response resp;
                    if (client.call(req, resp))
                        _context->yoloCallback(resp);
                    else
                        ROS_ERROR("detector service error");
                }
                else
                    ROS_ERROR("camera service error");
            }
            return tail;
        }

        void update()
        {
            if (this->_context->detector_flag)
            {
                ROS_INFO("execute");
                TransState("Auto");
            }
            else
                ROS_INFO("detecting...");
        }

    private:
        ros::NodeHandle n_;
        bool camera_state_ = false;
        ros::ServiceClient client;
        ros::ServiceClient color_map_client_;
        ros::ServiceClient depth_map_client_;
    };

}

#endif
