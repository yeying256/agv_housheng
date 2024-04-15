#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <thread>
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
#include "hfsm/GohomeState.h"

#include "signal.h"
#include "geometry_msgs/Twist.h"

using namespace hfsm_ns;

bool run = true;

void hardwareLoggerCallback(const agv_msg::error_log &msg)
{
    ROS_INFO("%s", msg.error_msg);
}

xj_dy_ns::Eigen2file vel_log_;
void cmd_vel_cb(const geometry_msgs::Twist& msg)
{
    Eigen::VectorXd vel(3);
    vel(0)=msg.linear.x;
    vel(1)=msg.linear.y;
    vel(2)=msg.angular.z;

    // vel_log_.data_in(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hfsm_node");
    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::NodeHandle nh;

    // 记录数据用
    // vel_log_.init("/home/easycell/catkin_ws/data/","cmd_vel.csv");
    // std::vector<std::string> name;
    // name.push_back("vx");
    // name.push_back("vy");
    // name.push_back("vw");
    // vel_log_.title(name);


    Context *context = new Context(nh);

    ros::Subscriber sub = nh.subscribe("/push_button", 10, &Context::buttonCallback, context);
    ros::Subscriber goal_sub = nh.subscribe("move_base/result", 10, &Context::statusCallback, context);
    // ros::Subscriber yolo_sub = nh.subscribe("/DetectorResult_", 1, &Context::yoloCallback, context);
    ros::Subscriber hw_error_sub = nh.subscribe("/hardware_logger", 10, hardwareLoggerCallback);
    // ros::Subscriber cmd_vel_datalog = nh.subscribe("/cmd_vel",10,cmd_vel_cb);

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
    State *gohome = new GohomeState();
    context->CreateState(gohome, "Gohome");

    // 开始状态机
    sleep(2);
    context->Start("HardwareInit");

    int time = 0;
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
                // nav_flag :导航阶段
                // take_flag： 手上有没有料

                if (context->nav_flag == 0)
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
                    // continue;
                    // context->take_flag = 0;


                }

                // !!!!!!!!!!!!!!!!!!!!!测试用
                context->nav_flag = 1;
                context->take_flag = 1;
                
                // 如果前面运行过了，并且拿起来了
                if (context->nav_flag == 1)
                {
                    // 如果
                    if(context->take_flag == 1)
                    {
                        // 放料
                        EventData e = EventData((int)put_);

                        // 把这个消息发过去
                        std::vector<geometry_msgs::Pose> way_point;
                        geometry_msgs::Pose point1;

                        point1.position.x = 2.546320548332148;
                        point1.position.y = -1.4942356780273902;
                        point1.orientation.x = 0.0;
                        point1.orientation.y = 0.0;
                        point1.orientation.z = 0.5748077291939767;
                        point1.orientation.w = 0.8182885031936254;

                        way_point.push_back(point1);

                        // 第二个点：原地转个圈


                        point1.position.x = 3.243672395914593;
                        point1.position.y = -1.6305395628278034;
                        point1.orientation.z = -0.1700017543506582;
                        point1.orientation.w = 0.9854437596929103;

                        way_point.push_back(point1);

                        point1.position.x = 10.325582067881982;
                        point1.position.y = -3.6231888689456717;
                        point1.orientation.z = -0.17284721479546436;
                        point1.orientation.w = 0.984948648578925;

                        way_point.push_back(point1);

                        point1.position.x = 10.325582067881982;
                        point1.position.y = -3.6231888689456717;
                        point1.orientation.z = 0.5748077291939767;
                        point1.orientation.w = 0.8182885031936254;

                        way_point.push_back(point1);

                        // 有几个途径点就写几个

                        e.SetData(&way_point);
                        context->SendEvent(e);

                        // 动夹爪
                        e = EventData((int)automove_grabmove_);
                        grab_move gb_msg;
                        e.SetData(&gb_msg);
                        context->SendEvent(e);

                        // 退出
                        e = EventData((int)put_);
                        way_point.clear();

                        point1.position.x = 2.6432228129606186;
                        point1.position.y = -1.6002449424544825;
                        point1.orientation.z = 0.9883107167820162;
                        point1.orientation.w = 0.15245303241922506;

                        way_point.push_back(point1);

                        point1.position.x = 0.0;
                        point1.position.y = 0.0;
                        point1.orientation.z = 0.0;
                        point1.orientation.w = 1.0;

                        way_point.push_back(point1);

                        e.SetData(&way_point);
                        context->SendEvent(e);

                        // 重置takeflag
                        context->take_flag = 0;
                    }
                    context->nav_flag = 2;
                }
                // !!!!!!!!!!!!!!!!!!!!!!测试用循环跑
                    context->nav_flag = 0;
                // !!!!!!!!!!!!!!!!!!!!!!测试用



        }
        else if (context->GetCurStateName() == "Adjust")
        {
            EventData e = EventData((int)task_);
            ar_data send;
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
        else if (current_state == "Gohome")
        {
			context->TransForState("Auto");
            geometry_msgs::Quaternion q;
            q.w = 1;
            q.z = 0;
            q.y = 0;
            q.x = 0;
            EventData e = EventData((int)go_);
            goal_data home = {0, 0, q};
            e.SetData(&home);
            context->SendEvent(e);
			context->TransForState("Idle");

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
