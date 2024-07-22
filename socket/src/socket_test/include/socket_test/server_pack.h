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

class ServerPack
{
public:
    ServerPack(int16_t port,bool open_crc,int timeout);
    ~ServerPack();
    void handle_udp_msg(int fd,bool open_crc);
    void run();

private:
    int server_fd,ret;
    int timeout;
    struct sockaddr_in ser_addr;    //地址

    void handle_date(char *buf);

    int16_t get_crc16_value(uint8_t *data, uint16_t len)
    {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < len; i++)
        {
            crc ^= data[i];     //低8位异或
            for (uint16_t j = 0; j < 8; j++)
            {
                if (crc & 0x01)
                {
                    crc = (crc >> 1) ^ 0xA001;  //右移后与多项式反转后异或
                }
                else
                {
                    crc = crc >> 1;
                }
            }
        }
        return crc^0xFFFF;
    }

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

