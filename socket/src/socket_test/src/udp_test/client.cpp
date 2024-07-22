#include<sys/select.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>

union client_pack
{
    char buffer[3];
    int data;
}uDirection, uAccel_up, uAccel_dowm;


void udp_msg_send(int fd, struct sockaddr* dst)
{
    char buf[BUFFER_LEN]={0};
    socklen_t socket_len;
    struct sockaddr_in src;
    int direction=30, accel_up=4, accel_down=1;

    uDirection.data = direction;
    uAccel_up.data = accel_up;
    uAccel_dowm.data = accel_down;

    for(int i=0; i<3; i++)
        buf[i] = uDirection.buffer[i];
    for(int i=3; i<6; i++)
        buf[i] = uAccel_up.buffer[i-3];
    for(int i=6; i<9; i++)
        buf[i] = uAccel_dowm.buffer[i-6];

    while(1)
    {
        socket_len = sizeof(*dst);
        sendto(fd,buf,BUFFER_LEN, 0, dst, socket_len);
        sleep(1);   //休眠一秒
    }
}



int main(int argc, char *argv[])
{
    int client_fd;
    struct sockaddr_in ser_addr;    // 发送给哪个主机的地址，给server
    
    client_fd = socket(AF_INET, SOCK_DGRAM, 0);  //创建socket
    if(client_fd < 0)
    {
        perror("create socket fail!\n");
        return -1;
    }

    ser_addr.sin_family = AF_INET;
    ser_addr.sin_port = htons(SERVER_PORT);
    // ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    ser_addr.sin_addr.s_addr = htons(INADDR_ANY);

    udp_msg_send(client_fd, (struct sockaddr*)&ser_addr);

    std::cout << "client end!" << std::endl;
    close(client_fd);

    return 0;
}
