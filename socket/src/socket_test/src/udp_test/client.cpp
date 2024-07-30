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
#include "ros/ros.h"
#include "socket_test/server_pack.h"
#include "std_msgs/Float64.h"
#include "dynamic_reconfigure/server.h"
#include "socket_test/drConfig.h"

#define BUFFER_LEN 16
#define SERVER_PORT 6050

int throttle = 0, brake = 0, dangwei = 0, engine = 0;
bool addThrottle = false;
void cb(demo02_dr::drConfig& config, uint32_t level){
    throttle = config.throttle;
    brake = config.brake; 
    dangwei = config.car_class;
    addThrottle = config.addThrottle;
    engine = config.engine;
    ROS_INFO("动态参数解析数据:%d,%d,%d,%s,%d",
        config.brake,
        config.throttle,
        config.addThrottle,
        config.string_param.c_str(),
        config.car_class
    );
}

void udp_msg_send(int fd, struct sockaddr* dst)
{
    char buf[BUFFER_LEN]={0};
    socklen_t socket_len;
    struct sockaddr_in src;
    int direction=30;
     
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        //数据打包成字符串，一个数据三位
        sprintf(buf,"%03d%03d%03d%01d%01d%01d",direction,throttle,brake,dangwei,addThrottle,engine);
        socket_len = sizeof(*dst);
        sendto(fd,buf,BUFFER_LEN, 0, dst, socket_len);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
// F1Car car(10, 10, 50, 0.3, 1.225 ,1.5);
// ros::Publisher speed_pub;
// ros::Publisher accel_pub;

// void control_timer_callback(const ros::TimerEvent& event)
// {
//     static ros::Time last_time = event.last_real;
//     ros::Time current_time = event.current_real;
//     double dt = (current_time - last_time).toSec();
//     std::cout << "timer callback" << std::endl;
//     if(dt<1)    //第一次的dt错误
//         car.UpdateSpeed(throttle,brake,dt);
//     std::cout << "speed: " << car.GetSpeed() << " accel: " << car.GetAccel() << std::endl;
//     last_time = current_time;

//     std_msgs::Float64 speed_msg;
//     std_msgs::Float64 accel_msg;
//     speed_msg.data = car.GetSpeed();
//     accel_msg.data = car.GetAccel();
//     speed_pub.publish(speed_msg);
//     accel_pub.publish(accel_msg);
// }


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<demo02_dr::drConfig> server;
    // 4.创建回调对象(使用回调函数，打印修改后的参数)
    dynamic_reconfigure::Server<demo02_dr::drConfig>::CallbackType cbType;
    cbType = boost::bind(&cb,_1,_2);
    // 5.服务器对象调用回调对象
    server.setCallback(cbType);

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
    // ros::spin();
    udp_msg_send(client_fd, (struct sockaddr*)&ser_addr);
    
    std::cout << "client end!" << std::endl;
    close(client_fd);

    // speed_pub = nh.advertise<std_msgs::Float64>("speed", 1);
    // accel_pub = nh.advertise<std_msgs::Float64>("accel", 1);
    // ros::Timer timer = nh.createTimer(ros::Duration(0.01),control_timer_callback);

    return 0;
}
