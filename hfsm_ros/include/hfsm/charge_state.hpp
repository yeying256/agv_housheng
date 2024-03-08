#ifndef CHARGE_STATE_HPP
#define CHARGE_STATE_HPP

#include <hfsm/Context.h>
#include <hfsm/State.h>
#include <hfsm/param.h>
#include <hfsm/tcp_client.hpp>
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
    private:
        TcpClient *tcpClient;
        uint8_t start_charge[6] = {0x01, 0x05, 0x00, 0x07, 0xFF, 0x00};
        uint8_t stop_charge[6] = {0x01, 0x05, 0x00, 0x08, 0xFF, 0x00};
        uint8_t check_station[6] = {0x01, 0x01, 0x00, 0x40, 0x00, 0x18};
        uint8_t read_volt[6] = {0x01, 0x03, 0x00, 0x65, 0x00, 0x06};

    public:
        void start();
        void stop();
        int send(uint8_t *cmd, uint8_t len_cmd);
    };
    void ChargeState::start()
    {
        printf_yellow("ChargeState start");
        tcpClient = new TcpClient();
        if (!tcpClient->clientInit())
        {
            ROS_ERROR("charging station connect failed");
            return;
        }
        ros::Duration(0.5).sleep();
        uint8_t cmd_len = sizeof(check_station) / sizeof(check_station[0]);
        if (!send(check_station,cmd_len))
        {
            ros::Duration(0.5).sleep();
            uint8_t recv_buf[RECV_BUF_SIZE] = {0};
            if (tcpClient->clientReceive(recv_buf))
                if ((*(recv_buf + 3) & 0xff) == 0xd1)
                {
                    ros::Duration(0.5).sleep();
                    cmd_len = sizeof(start_charge) / sizeof(start_charge[0]);
                    send(start_charge, cmd_len);
                }
        }
        else
            ROS_ERROR("charge station check failed");
    }

    void ChargeState::stop()
    {
        printf_green("ChargeState stop");
    }

    int ChargeState::send(uint8_t *cmd, uint8_t len_cmd)
    {
        uint16_t ret = tcpClient->crc16(cmd, 6);
        uint8_t high = ret >> 8;
        uint8_t low = ret & 0x00ff;
        uint8_t crc[] = {low, high};
        auto *msg = (char *)malloc(len_cmd + sizeof(crc) / sizeof(crc[0]));
        memcpy(msg, cmd, len_cmd);
        memcpy(msg + len_cmd, crc, sizeof(crc) / sizeof(crc[0]));
        if (tcpClient->clientSend(msg))
            return 0;
        else
        {
            ROS_INFO("charging station message send failed");
            return 1;
        }
    }

}

#endif
