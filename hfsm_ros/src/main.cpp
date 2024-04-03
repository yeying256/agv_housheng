#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <agv_msg/Button.h>
#include <agv_msg/reltive_pose.h>
#include <agv_msg/error_log.h>
#include <unordered_map>
#include <agv_msg/grab_agv.h>
#include <agv_msg/reltive_pose_visual.h>
#include <agv_msg/visual_point_move.h>
#include <std_msgs/String.h>

#include <hfsm/ShutdownState.h>
#include "hfsm/AdjustState.h"

#include "hfsm/StartState.h"
#include "hfsm/idle_state.hpp"
#include "hfsm/charge_state.hpp"
#include "hfsm/TelecontrolState.h"
#include "hfsm/AutoState.h"
#include "hfsm/hardware_init_state.hpp"

#include "signal.h"

using namespace hfsm_ns;

bool run = true;

void hardwareLoggerCallback(const agv_msg::error_log &msg)
{
    ROS_INFO("%s", msg.error_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hfsm_node");
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::NodeHandle nh;

    Context *context = new Context(nh);
    ros::Subscriber sub = nh.subscribe("/push_button", 10, &Context::buttonCallback, context);
    ros::Subscriber goal_sub = nh.subscribe("move_base/result", 10, &Context::statusCallback, context);
    ros::Subscriber yolo_sub = nh.subscribe("/DetectorResult_", 10, &Context::yoloCallback, context);
    ros::Subscriber hw_error_sub = nh.subscribe("/hardware_logger", 10, hardwareLoggerCallback);

    // 创建状态机
    State *hardware_init = new HardwareInitState();
    context->CreateState(hardware_init, "HardwareInit");
    State *start = new StartState();
    context->CreateState(start, "Start");
    State *idle = new IdleState();
    context->CreateState(idle, "Idle");
    State *tele = new TelecontrolState();
    context->CreateState(tele, "Telecontrol");
    State *charge = new ChargeState();
    context->CreateState(charge, "Charge");
    State *automation = new AutoState();
    context->CreateState(automation, "Auto");
    State *adjust = new AdjustState(nh);
    context->CreateState(adjust, "Adjust", "Idle");
    State *shutdown = new ShutdownState();
    context->CreateState(shutdown, "Shutdown");

    // 开始状态机
    sleep(2);
    context->Start("HardwareInit");

    int time = 0;
    int way_point = 1;
    int detect_times = 0;
    while (ros::ok() && (context->run_ == true))
    {
        time++;
        ros::Duration(0.1).sleep();
        // std::cout<<"现在进入了主循环"<<std::endl;

        // std::cout<<"context->GetCurStateName() = "<<context->GetCurStateName()<<std::endl;

        std::string current_state = context->GetCurStateName();

        // 如果为自动状态，每隔1s发出心跳事件
        if (current_state == "Auto")
        {
            // std::cout<<"现在进入了auto"<<std::endl;
            if (time % 10 == 0)
            {
                EventData e = EventData((int)checkheart_);
                context->SendEvent(e);
            }
            // wangxiao

            // wangxiao

            if (context->nav_flag == 0)
            {
                if (way_point == 1)
                {
                    std::cout << "send waypoint" << std::endl;
                    // 途经点
                    // 		x: 3.060587364997074
                    //   y: -0.5225231791775863
                    //   z: 0.0
                    // orientation: 
                    //   x: 0.0
                    //   y: 0.0
                    //   z: 0.5805667186466207
                    //   w: 0.8142126781129704

                    geometry_msgs::Quaternion q;
                    q.w = 0.8130156795809587;
                    q.z = 0.5822417923470556;
                    q.y = 0;
                    q.x = 0;
                    EventData e = EventData((int)go_);
                    goal_data put = {2.91035048977949, -0.4558212749612451, q};
                    e.SetData(&put);
                    context->SendEvent(e);
                    way_point = 0;
                    // continue;
                    // context->take_flag = 0;
                }

                // if (context->take_flag == 1)
                // {
                //     // 放
                //     // position:
                //     // x: 4.201893556236952
                //     // y: -0.9959773657127725
                //     // z: 0.0
                //     // orientation:
                //     // x: 0.0
                //     // y: 0.0
                //     // z: -0.5843139394532142
                //     // w: 0.811527707574218

                //     geometry_msgs::Quaternion q;
                //     q.w = 0.811527707574218;
                //     q.z = -0.5843139394532142;
                //     q.y = 0;
                //     q.x = 0;
                //     EventData e = EventData((int)go_);
                //     goal_data put = {4.201893556236952, -0.9959773657127725, q};
                //     e.SetData(&put);
                //     context->SendEvent(e);
                //     // context->take_flag = 0;
                // }

                // if (context->put_flag == 1)
                // {
                //     // 放
                //     // position:
                //     //   x: 4.441739180668326
                //     //   y: -1.5032635120511704
                //     //   z: 0.0
                //     // orientation:
                //     //   x: 0.0
                //     //   y: 0.0
                //     //   z: -0.5738386402429688
                //     //   w: 0.8189683845937525
                //     std::cout << "go to put down" << std::endl;

                //     geometry_msgs::Quaternion q;
                //     q.w = 0.8241698824972953;
                //     q.z = -0.5663426566879758;
                //     q.y = 0;
                //     q.x = 0;
                //     EventData e = EventData((int)go_);
                //     goal_data start = {10.870451444661438, 4.875714719729624, q};
                //     e.SetData(&start);
                //     context->SendEvent(e);
                //     // context->take_flag = 0;
                // }
            }

            // if (context->nav.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && way_point == 1)
            // {
            //     way_point = 0;
            //     continue;
            // }

            // if (context->nav.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // {
            //     context->nav_flag = 0;

            //     context->TransForState("Adjust");
            // }

            // 前面导航进入取料位置
            // if (context->nav_flag == 0)
            // {
            //     context->nav_flag = 1;
            // }

            // 如果前面运行过了，并且拿起来了
            if (context->nav_flag == 1)
            {
                // 如果
                // if(context->take_flag == 1)
                {
                    // 放料
                    EventData e = EventData((int)put_);

                    // 把这个消息发过去
                    std::vector<geometry_msgs::Pose> way_point;
                    geometry_msgs::Pose point1;
                    // 第一个点，退出
                    point1.position.x = 2.8881201451787653;
                    point1.position.y = 3.2236157804702135;
                    point1.orientation.x = 0.0;
                    point1.orientation.y = 0.0;
                    point1.orientation.z = 0.6292847017063496;
                    point1.orientation.w = 0.7771748607606596;

                    way_point.push_back(point1);

                    // 第二个点：原地转个圈
                    point1.orientation.z = -0.8033431988003251;
                    point1.orientation.w = 0.5955163347392424;

                    way_point.push_back(point1);

                    // 有几个途径点就写几个

                    e.SetData(&way_point);
                    context->SendEvent(e);

                    // 动夹爪
                    // e = EventData((int)automove_grabmove_);
                    // grab_move gb_msg;
                    // e.SetData(&gb_msg);
                    // context->SendEvent(e);

                    // 退出
                    way_point.clear();

                    // e.SetData(&way_point);
                    // context->SendEvent(e);

                    // 重置takeflag
                    context->take_flag = 0;
                }

                context->nav_flag = 2;
            }
        }
        else if (context->GetCurStateName() == "Adjust")
        {
            EventData e = EventData((int)task_);
            ar_data send;
            // if (context->take_flag == 0)
            // pre hight width
            // pre_height – 预备状态夹爪的高度
            // pre_width – 预备状态夹爪宽度
            // grab_width – 动作的时候夹爪的宽度
            // grab_height – 动作时候夹爪的高度
            // workbench_id
            send = {850, 950, 1000, 950, 0};
            e.SetData(&send);
            context->SendEvent(e);
        }
        else if (current_state == "Idle")
        {
            if (!context->detector_flag)
            {
                ROS_INFO("take photo");
                EventData e = EventData((int)photo_);
                context->SendEvent(e);
                ros::Duration(1.0).sleep();
            }
        }
    }

    if (context)
    {
        delete context;
        context = nullptr;
        delete start;
        delete idle;
        delete tele;
        delete automation;
        delete shutdown;
        // system("roslaunch agvsim_navigation nav_vision.launch --shutdown");
    }
    spinner.stop();
    return 0;
}
