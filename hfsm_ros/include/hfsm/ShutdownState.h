#ifndef SHUTDOWNSTATE_H
#define SHUTDOWNSTATE_H

#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <agv_msg/Button.h>
#include <agv_msg/reltive_pose.h>
#include <unordered_map>
#include <agv_msg/grab_agv.h>
#include <agv_msg/reltive_pose_visual.h>
#include <agv_msg/visual_point_move.h>



namespace hfsm_ns
{
    class ShutdownState : public State
    {
    public:
        void start();
    };

}

#endif