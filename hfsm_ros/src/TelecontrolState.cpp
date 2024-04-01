/*
 * @Author: LHYLHYY 876814061@qq.com
 * @Date: 2024-01-30 18:12:50
 * @LastEditors: LHYLHYY 876814061@qq.com
 * @LastEditTime: 2024-03-31 15:19:06
 * @FilePath: /catkin_ws/src/agv_sim/hfsm_ros/src/TelecontrolState.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "hfsm/TelecontrolState.h"

namespace hfsm_ns{
    void TelecontrolState::start()
    {
        printf_yellow("TelecontrolState start");
    }

    void TelecontrolState::stop()
    {
        printf_green("TelecontrolState stop");
    }

    void TelecontrolState::update()
    {
        TransState("Auto");
    }
}