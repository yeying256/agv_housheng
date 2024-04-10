/*
 * @Author: LHYLHYY 876814061@qq.com
 * @Date: 2024-01-30 18:12:50
 * @LastEditors: LHYLHYY 876814061@qq.com
 * @LastEditTime: 2024-04-05 16:24:51
 * @FilePath: /catkin_ws/src/agv_sim/hfsm_ros/src/TelecontrolState.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "hfsm/GohomeState.h"

namespace hfsm_ns{
    void GohomeState::start()
    {
        printf_yellow("GohomeState start");
        bool single_flag = 0;

        bool detector_flag = 0;

        int take_flag = 0;

        int put_flag = 0;

        int nav_flag = 0; // flag = 0的时候是导航结束，到位置了，可以发布下一个命令了
    }

    void GohomeState::stop()
    {
        printf_green("GohomeState stop");
    }

    void GohomeState::update()
    {
        TransState("Auto");
    }
}