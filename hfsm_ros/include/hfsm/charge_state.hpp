#ifndef CHARGE_STATE_HPP
#define CHARGE_STATE_HPP

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
    class ChargeState : public State
    {
    public:
        void start()
        {
            printf_yellow("ChargeState start");
        }

        void stop()
        {
            printf_green("ChargeState stop");
        }
    };

}

#endif
