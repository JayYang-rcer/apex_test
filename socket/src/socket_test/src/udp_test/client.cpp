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


#define BUFFER_LEN 512
#define SERVER_PORT 8888
#define SERVER_IP "172.0.5.182"

void udp_msg_send(int fd, struct sockaddr* dst)
{
    socklen_t socket_len;
    struct sockaddr_in src;
    while(1)
    {
        char buf[BUFFER_LEN] = "TEST UPD MSG\n";
        socket_len = sizeof(*dst);
        std::cout << "client: " << buf << std::endl;
        sendto(fd,buf,BUFFER_LEN, 0, dst, socket_len);

        recvfrom(fd, buf, BUFFER_LEN, 0, (struct sockaddr*)&src, &socket_len);  //接收来自server的信息
        std::cout << "server: " << buf << std::endl;
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
    close(client_fd);

    return 0;
}
