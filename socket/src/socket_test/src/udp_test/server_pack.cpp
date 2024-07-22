#include "socket_test/server_pack.h"

#define BUFFER_LEN 16

union server_pack
{
    char buffer[3];
    int data;
}direction, accel_up, accel_dowm;


/**
 * @brief Construct a new Server Pack:: Server Pack object
*/
ServerPack::ServerPack(int16_t port, int timeout)
{
    this->timeout_ = timeout_;
    server_fd_ = socket(AF_INET, SOCK_DGRAM, 0);  //创建socket
    if(server_fd_ < 0)
    {
        perror("create socket fail!\n");
        return;
    }

    //设置地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

     struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = timeout;  // 200 ms
    setsockopt(server_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));

    result_ = bind(server_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));   //绑定地址结构体和socket
    if(result_ < 0)
    {
        perror("bind fail!\n");
        return;
    }
}

ServerPack::~ServerPack()
{
    close(server_fd_);
}


/**
 * @brief 数据处理函数
 * 
 * @param buf 

*/
void ServerPack::HandleData(char *data)
{
    std::cout << "data: " << data << std::endl;
    for(int i=0; i<3; i++)
        direction.buffer[i] = data[i];
    for(int i=3; i<6; i++)
        accel_up.buffer[i-3] = data[i];
    for(int i=6; i<9; i++)
        accel_dowm.buffer[i-6] = data[i];

    direction_ = direction.data;
    accel_up_ = accel_up.data;
    accel_dowm_ = accel_dowm.data;
    std::cout << "direction: " << direction_ << " accel_up: " << accel_up_ << " accel_dowm: " << accel_dowm_ << std::endl;
}


/**
 * @brief 处理udp消息
*/
void ServerPack::HandleMsg(int fd)
{
    char buf[BUFFER_LEN];   //接收缓冲区，16字节
    struct sockaddr_in clent_addr;  //clent_addr用于记录发送方的地址信息
    socklen_t len;
    int count;

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
            
            HandleData(buf);
            memset(buf, 0, BUFFER_LEN); //清空buf
        }
    }
}


void ServerPack::Run()
{
    HandleMsg(server_fd_);
}


int main(int argc, char *argv[])
{
    ServerPack server(8888,200000*5);
    server.Run();
    return 0;
}

