#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/SendGoal.hpp>
#include <hfsm/param.h>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <agv_msg/Button.h>
#include <agv_msg/reltive_pose.h>
#include <unordered_map>

bool run = true;

// 开始状态
class StartState : public State
{
public:
    void start()
    {
        std::cout << "StartState start" << std::endl;
    }

    void stop()
    {
        std::cout << "StartState stop" << std::endl;
    }

    void update()
    {
        TransState("Lock");
    }
};

// 锁定状态（不可移动状态）
class LockState : public State
{
public:
    void start()
    {
        std::cout << "LockState start" << std::endl;
    }

    void stop()
    {
        std::cout << "LockState stop" << std::endl;
    }

    void update()
    {
        TransState("Telecontrol");
    }
};

// 遥控状态
class TelecontrolState : public State
{
public:
    void start()
    {
        std::cout << "TelecontrolState start" << std::endl;
    }

    void stop()
    {
        std::cout << "TelecontrolState stop" << std::endl;
    }

    void update()
    {
        TransState("Auto");
    }
};

// 自动状态 （ROS接管状态）
class AutoState : public State
{
public:
    NavState nav;
    void start()
    {
        ROS_INFO("AutoState start");
        // nav.set_goal(10, 10, 1);
        std::function<EventDeal(EventData &)> func = std::bind(&AutoState::DealEvent, this, std::placeholders::_1);
        set_event_func(func);
    }

    EventDeal DealEvent(EventData &event_data)
    {
        if ((EventS)event_data._event_type == checkheart_)
        {
            ROS_INFO("AUTO WORKING");
        }
        if ((EventS)event_data._event_type == go_)
        {
            ROS_INFO("send goal");
            goal_data *goal = event_data.GetData<goal_data>();
            int ret = nav.set_goal(goal->set_x, goal->set_y, goal->orientation);
            if (ret)
                TransState("Adjust");
        }
        return tail;
    }

    void stop()
    {
        std::cout << "AutoState stop" << std::endl;
    }

    void update()
    {
        ROS_INFO("WAITING");
    }
};

// 自动状态-调整状态
class AdjustState : public AutoState
{
public:
    ros::NodeHandle n;
    ros::ServiceClient client;
    agv_msg::reltive_pose::Request req;
    agv_msg::reltive_pose::Response resp;
    std::unordered_map<int8_t, workbench> workbench_map;
    AdjustState(ros::NodeHandle nh) : n(nh) { client = nh.serviceClient<agv_msg::reltive_pose>("ar_track_target"); };
    void start()
    {
        ROS_INFO("Adjust start");
    }

    void stop()
    {
        ROS_INFO("Adjust stop");
    }

    void update()
    {
        ROS_INFO("SEARCHING FOR AR MARK...");
        workbench find_ar = workbench_map.find(1)->second;
        req.pose.x = find_ar.bias_x;
        req.pose.y = find_ar.bias_y;
        req.pose.theta = 0;
        bool ok = client.call(req, resp);
        if (ok)
        {
            ROS_INFO("client: send bias = %f, bias = %f", req.pose.x, req.pose.y);
        }
        else
        {
            ROS_ERROR("failed to send ar_target service");
        }
        TransState("Auto");
    }
};

// 关机状态
class ShutdownState : public State
{
public:
    void start()
    {
        std::cout << "ShutdownState start" << std::endl;
    }

    void stop()
    {
        std::cout << "ShutdownState stop" << std::endl;
    }

    void update()
    {
        ROS_INFO("agent shutdown");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hfsm_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    Context *context = new Context();
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/push_button", 100, &Context::ButtonCallback, context);

    // 创建状态机
    State *start = new StartState();
    context->CreateState(start, "Start");
    State *lock = new LockState();
    context->CreateState(lock, "Lock");
    State *tele = new TelecontrolState();
    context->CreateState(tele, "Telecontrol");
    State *automation = new AutoState();
    context->CreateState(automation, "Auto");
    State *adjust = new AdjustState(nh);
    context->CreateState(adjust, "Adjust");
    State *shutdown = new ShutdownState();
    context->CreateState(shutdown, "Shutdown");

    // 开始状态机
    context->Start("Start");

    int time = 0;
    while (ros::ok())
    {
        time++;
        ros::Duration(0.1).sleep();

        // 如果为自动状态，每隔1s发出心跳事件
        if (context->GetCurStateName() == "Auto" && time % 10 == 0)
        {
            EventData e = EventData((int)checkheart_);
            context->SendEvent(e);
        }
    }

    if (context)
    {
        delete context;
        context = nullptr;
        delete start;
        delete lock;
        delete tele;
        delete automation;
        delete shutdown;
    }

    std::cout << "state close" << std::endl;
    spinner.stop();
    return 0;
}
