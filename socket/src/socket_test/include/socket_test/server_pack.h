#pragma once

#include "sys/select.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/socket.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>

#define BUFFER_LEN 16
#define SERVER_PORT 8888
#define SERVER_IP "172.0.5.182"

class ServerPack
{
public:
    ServerPack(int16_t port,int timeout);
    ~ServerPack();
    void HandleMsg(int fd);
    void Run();

private:
    int server_fd_,result_;
    int timeout_;
    float direction_, accel_up_,accel_dowm_;    //方向，加速度, 减速度
    struct sockaddr_in server_addr;    //地址

    void HandleData(char *data);

    uint8_t get_crc8_value(uint8_t *data, uint8_t len)
    {
        uint8_t crc = 0;
        uint8_t i;
        while(len--)
        {
            crc ^= *data++;
            for(i = 0; i < 8; i++)
            {
                if(crc&0x01)
                    crc=(crc>>1)^0x8C;
                else
                    crc >>= 1;
            }
        }
        return crc;
    }
};

