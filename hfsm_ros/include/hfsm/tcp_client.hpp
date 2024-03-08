#ifndef _TCP_CLIENT_HPP_
#define _TCP_CLIENT_HPP_

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string.h>

#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 3000
#define RECV_BUF_SIZE 18

class TcpClient
{
private:
    int client_fd = -1;
    std::string server_ip;
    unsigned int server_port = -1;

public:
    TcpClient() : server_ip(SERVER_ADDRESS), server_port(SERVER_PORT){};
    ~TcpClient();
    bool clientInit();
    bool clientSend(const char *data);
    bool clientReceive(uint8_t *recv_buf);
    uint16_t crc16(const uint8_t *ptr, uint8_t len);
};

TcpClient::~TcpClient()
{ // close socket
    if (client_fd != -1)
        close(client_fd);
}

bool TcpClient::clientInit()
{ // create socket
    client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd == -1)
    {
        std::cout << "create client fd error. " << std::endl;
        return false;
    }

    // connect to server
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    server_addr.sin_port = htons(SERVER_PORT);
    if (connect(client_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        std::cout << "connect socket error" << std::endl;
        return false;
    }
    return true;
}

bool TcpClient::clientSend(const char *data)
{
    if (client_fd == -1)
        return false;
    // send data to server
    int ret = send(client_fd, data, strlen(data), 0);
    if (ret != strlen(data))
    {
        std::cout << "send data error." << std::endl;
        return false;
    }
    std::cout << "send data successfully, data :" << data << std::endl;
    return true;
}

bool TcpClient::clientReceive(uint8_t *msg)
{
    if (client_fd == -1)
        return false;
    // client receive data
    uint8_t *recv_buf;
    int ret = recv(client_fd, recv_buf, RECV_BUF_SIZE, 0);
    if (ret > 0)
    {
        for (int i = RECV_BUF_SIZE; i > 0; i--)
        {
            if (*(recv_buf + i) != 0)
            {
                uint8_t high = *(recv_buf + i - 1);
                uint8_t low = *(recv_buf + i - 2);
                uint16_t msg_crc16 = high << 8 & low;
                memcpy(msg, recv_buf, i - 2);
                uint16_t calc_crc16 = crc16(msg, i - 2);
                if (calc_crc16 == msg_crc16)
                    return true;
                else
                {
                    ROS_ERROR("CRC16 check error");
                    return false;
                }
            }
        }
    }

    ROS_ERROR("empty receive");

    return false;
}

uint16_t TcpClient::crc16(const uint8_t *ptr, uint8_t len)
{ // crc16 check
    uint16_t tmp, CRC16;

    CRC16 = 0xffff;
    for (uint8_t i = 0; i < len; i++)
    {
        CRC16 = *ptr ^ CRC16;
        for (uint8_t j = 0; j < 8; j++)
        {
            tmp = CRC16 & 0x0001;
            CRC16 = CRC16 >> 1;
            if (tmp)
                CRC16 = CRC16 ^ 0xa001;
        }
        *ptr++;
    }
    return CRC16;
}

#endif
