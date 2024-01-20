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

namespace hfsm_ns
{
    // 硬件初始化状态
    class IdleState : public State
    {
    public:
        void start()
        {
            printf_yellow("IdleState start");
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
        }

        void stop()
        {
            printf_green("IdleState stop");
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
        ros::ServiceClient color_map_client_ = n_.serviceClient<mecheye_ros_interface::CaptureColorMap>("capture_color_map");
        ros::ServiceClient depth_map_client_ = n_.serviceClient<mecheye_ros_interface::CaptureDepthMap>("capture_depth_map");
        bool camera_state_ = false;
    };

}

#endif
