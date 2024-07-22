#include "socket_test/server_pack.h"

#define BUFFER_LEN 1024

/**
 * @brief Construct a new Server Pack:: Server Pack object
*/
ServerPack::ServerPack(int16_t port,bool open_crc, int timeout)
{
    this->timeout = timeout;
    server_fd = socket(AF_INET, SOCK_DGRAM, 0);  //创建socket
    if(server_fd < 0)
    {
        perror("create socket fail!\n");
        return;
    }

    //设置地址
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_port = htons(port);
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);

     struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = timeout;  // 200 ms
    setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));

    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));   //绑定地址结构体和socket
    if(ret < 0)
    {
        perror("bind fail!\n");
        return;
    }
}

ServerPack::~ServerPack()
{
    close(server_fd);
}


/**
 * @brief 数据处理函数
 * 
 * @param buf 

*/
void ServerPack::handle_date(char *buf)
{
    std::cout << "client: " << buf << std::endl;
}


/**
 * @brief 处理udp消息
*/
void ServerPack::handle_udp_msg(int fd, bool open_crc)
{
    char buf[BUFFER_LEN];   //接收缓冲区，1024字节
    socklen_t len;
    int count;
    struct sockaddr_in clent_addr;  //clent_addr用于记录发送方的地址信息

    while(1)
    {
        memset(buf, 0, BUFFER_LEN); //清空buf
        len = sizeof(clent_addr);   

        count = recvfrom(fd, buf, BUFFER_LEN, 0, (struct sockaddr*)&clent_addr, &len);  //接收数据
        if(count == -1)
        {
            std::cout << "recieve data fail!" << std::endl;
        }
        else
        {
            
            handle_date(buf);
            memset(buf, 0, BUFFER_LEN); //清空buf

            sprintf(buf, "recieve\n");    //回复client
            std::cout << "server: " << buf << std::endl;
            sendto(fd, buf, strlen(buf), 0, (struct sockaddr*)&clent_addr, len);   //发送信息给client
        }
    }
}


void ServerPack::run()
{
    handle_udp_msg(server_fd,false);
}


int main(int argc, char *argv[])
{
    ServerPack server(8888,false,200000*5);
    server.run();
    return 0;
}

