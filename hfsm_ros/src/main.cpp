#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/SendGoal.hpp>
#include <iostream>
#include <functional>
#include <vector>
#include <ros/ros.h>
#include <agv_msg/Button.h>

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
        time++;
        if (time == 10)
        {
            TransState("Lock");
        }
    }

    int time = 0;
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
        time++;
        if (time == 10)
        {
            TransState("Telecontrol");
        }
    }

    int time = 0;
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
        time++;
        if (time == 10)
        {
            TransState("Auto");
        }
    }

    int time = 0;
};

// 自动状态 （ROS接管状态）
class AutoState : public State
{
public:
    NavState nav;
    void start()
    {
        std::cout << "AutoState start" << std::endl;
        ROS_INFO("send simple goal");
        nav.set_goal(10, 10, 1);
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
            ROS_INFO("send simple goal");
            goal_data* goal = event_data.GetData<goal_data>();
            nav.set_goal(goal->set_x, goal->set_y, goal->orientation);
        }
        return tail;
    }

    void stop()
    {
        std::cout << "AutoState stop" << std::endl;
    }

    void update()
    {
        // TransState("Shutdown");
        ROS_INFO("WAITING");
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
        // TransState("ShutdownState");
        ROS_INFO("agent shutdown");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hfsm_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    Context *context = new Context();

    // 创建状态机
    State *start = new StartState();
    context->CreateState(start, "Start");
    State *lock = new LockState();
    context->CreateState(lock, "Lock");
    State *tele = new TelecontrolState();
    context->CreateState(tele, "Telecontrol");
    State *automation = new AutoState();
    context->CreateState(automation, "Auto");
    State *shutdown = new ShutdownState();
    context->CreateState(shutdown, "Shutdown");

    // 开始状态机
    context->Start("Start");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/push_button", 100, &Context::ButtonCallback, context);
    

    int time = 0;
    while (ros::ok())
    {
        time++;
        ros::Duration(0.1).sleep();
        // context->Update();

        // 如果为自动状态，每隔1s发出心跳事件
        if (context->GetCurStateName() == "Auto" && time % 10 == 0)
        {
            EventData e = EventData((int)checkheart_);
            context->SendEvent(e);
        }

        // if (context->GetCurStateName() == "Auto" && time % 1000 == 0)
        // {
        //     EventData e = EventData((int)go_);
        //     context->SendEvent(e);
        // }
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
