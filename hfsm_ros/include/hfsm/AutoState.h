#ifndef AUTPSTATE_H
#define AUTPSTATE_H
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
#include <hfsm/AdjustState.h>

// #define USE_MOVE_BASE



namespace hfsm_ns
{
    class AutoState : public State
    {
    public:
    // 取
    // position:
    //   x: 11.231310958376051
    //   y: 4.040175751665263
    //   z: 0.0
    // orientation:
    //   x: 0.0
    //   y: 0.0
    //   z: -0.5762847329167404
    //   w: 0.8172489869110154

    // 现场
    // ---
    // position:
    //     x: -4.487979013164646
    //     y: 3.4279136259250427
    //     z: 0.0
    // orientation:
    //     x: 0.0
    //     y: 0.0
    //     z: -0.9998475203881446
    //     w: 0.01746241603211958

public:
    void start();

    EventDeal DealEvent(EventData &event_data);
    bool grab_move(double width,double hight,bool width_move);
    // {
    //     if ((EventS)event_data._event_type == checkheart_)
    //     {
    //         ROS_INFO("AUTO WORKING");
    //     }
    //     if ((EventS)event_data._event_type == go_)
    //     {
    //         actionlib::SimpleClientGoalState state = this->_context->nav.ac.getState();

    //         agv_msg::grab_agv srv;
    //         srv.request.status = 4;
    //         if (this->_context->client.call(srv))
    //         {
    //             ROS_INFO("AGV MOVE!");
    //         }
    //         else
    //             ROS_INFO("AGV MOVE FAILED!");

    //         ROS_INFO("send goal");
    //         goal_data *goal = event_data.GetData<goal_data>();
    //         int ret = this->_context->nav.set_goal(goal->set_x, goal->set_y, goal->orientation);
    //         this->_context->nav_flag = 1;

    //         std::cout << "ac return " << state.toString() << std::endl;
    //     }
    //     if ((EventS)event_data._event_type == stop_)
    //     {
    //         ROS_INFO("stop navigation!");
    //         this->_context->nav.ac.cancelGoal();
    //         actionlib::SimpleClientGoalState state = this->_context->nav.ac.getState();
    //         std::cout << "current nav state is : " << state.toString() << std::endl;
    //         // Wait for the action to return
    //         this->_context->nav.ac.waitForResult();
    //         if (this->_context->nav.ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    //         {
    //             ROS_INFO("You have canceled the goal!");
    //         }
    //         else
    //         {
    //             ROS_ERROR("The cancel goal failed for some reason");
    //         }
    //     }
    //     return tail;
    // }

    void stop();
    // {
    //     printf_green("AutoState stop");
    // }

    void update();
    // {
    //     ROS_INFO("WAITING");
    //     TransState("Adjust");
    // }
};

}

#endif