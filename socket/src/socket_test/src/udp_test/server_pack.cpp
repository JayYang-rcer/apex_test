#include "socket_test/server_pack.h"
#include "fcntl.h"

#define BUFFER_LEN 16


/**
 * @brief Construct a new Server Pack:: Server Pack object
*/
ServerPack::ServerPack(int16_t port, int timeout)
{
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &ServerPack::OdomMsgCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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

    //配置为非阻塞接受
    int flags = fcntl(server_fd_, F_GETFL, 0);
    if(flags < 0)
    {
        perror("fcntl F_GETFL fail!\n");
        close(server_fd_);
        return;
    }

    if(fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        perror("fcntl F_SETFL fail!\n");
        close(server_fd_);
        return;
    }
}

ServerPack::~ServerPack()
{
    close(server_fd_);
}


void ServerPack::OdomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_ = *msg;
}


/**
 * @brief 数据处理函数
 * 
 * @param buf 

*/
void ServerPack::HandleData(char *data)
{
    std::cout << "data:" << data << std::endl;
    char change_data[12];
    for(int i=0; i<12; i++)
    {
        change_data[i] = data[i] - '0';
    }
    direction_ = change_data[0]*100 + change_data[1]*10 + change_data[2];
    throttle_ = change_data[3]*100 + change_data[4]*10 + change_data[5];
    brake_ = change_data[6]*100 + change_data[7]*10 + change_data[8];
    dangwei_ = change_data[9];
    add_throttle_ = change_data[10];
    engine_ = change_data[11];
    HandleAccel(throttle_, brake_,direction_);

    // std::cout << "direction: " << direction_ << " accel_up: " << throttle_ << " accel_down: " << brake_ << std::endl;
}


//油门和刹车都为0时，速度是衰减的。不给予速度指令，让速度自然衰减
//只踩油门时，加速度a的最大值a_max和速度上限是根据油门的大小来的，a_max = 系数k*油门。在油门不变时，加速度a会随着速度的增加而减小
//只踩刹车时，加速度a是一个固定值，不随速度的增加而改变
//踩油门和刹车时，优先级是刹车>油门，即当油门和刹车同时踩时，只有刹车起作用
void ServerPack::HandleAccel(int throttle, int brake, int direction)
{
    //将油门和刹车的值转换为0~1之间的值，0～655
    double throttle_f = throttle / 655.0;
    double brake_f = brake / 655.0;
    double direction_f = -(direction/655.0f - 0.5) * 3.14;
    static F1Car car(1.5, 0.3, 1.225 ,1.5,0.3,0.2);
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if(dt<1)    //第一次的dt错误
        car.UpdateSpeed(throttle_f,brake_f,dt);
    
    last_time = current_time;
    car.RealSpeedInput(odom_.twist.twist.linear.x);
    car.SetGear(dangwei_);
    car.AddThrottle(add_throttle_);
    car.EngineInput(engine_);

    cmd_vel_.linear.x = car.GetSpeed();
    cmd_vel_.angular.z =direction_f ;
    
    cmd_vel_pub_.publish(cmd_vel_);
    std::cout << "direction: " << direction<< " accel_up: " << throttle << " accel_down: " << brake << std::endl;
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

    ros::Rate loop_rate(100);
    static int timeout_cnt=0;
    while(ros::ok())
    {
        memset(buf, 0, BUFFER_LEN); //清空buf
        len = sizeof(clent_addr);   
        //  std::cout << "recieve data success!" << std::endl;
        count = recvfrom(fd, buf, BUFFER_LEN, 0, (struct sockaddr*)&clent_addr, &len);  //接收数据
        if(count == -1)
        {
            timeout_cnt++;
            if(timeout_cnt  > 200)
            {
                memset(buf, 0, BUFFER_LEN); //清空buf
                HandleAccel(0,0,0);
                std::cout << "recieve data fail!" << std::endl;
            }
        }
        else
        {
            // std::cout << "recieve data success!" << std::endl;
            timeout_cnt=0;
            HandleData(buf);
            memset(buf, 0, BUFFER_LEN); //清空buf
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void ServerPack::Run()
{
    HandleMsg(server_fd_);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "server_pack");
    ServerPack server(6050,0);
    server.Run();
    return 0;
}

