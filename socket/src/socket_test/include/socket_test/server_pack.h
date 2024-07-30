#pragma once
#include "ros/ros.h"
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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define BUFFER_LEN 16
#define SERVER_PORT 6050
#define SERVER_IP "172.0.5.182"


class F1Car
{
private:
    double speed_=0;  //当前速度
    double speed_max_;  //最大速度
    double accel_max_;  //最大加速度
    double accel_;  //当前加速度
    double decel_max_;  //最大减速度
    double Cd; //空气阻力系数
    double A;  //横截面积
    double rho;  //空气密度
    double car_mass=3;  //车重
    double real_speed_;
    double throttle_add_ = 1.0f;

    double CalculateAirResistance(double speed)
    {
        return 0.5 * Cd * A * rho * speed * speed;
        // return 0.5 * Cd * A * rho * speed_ * speed_;
    }

public:
    F1Car(double speed_max, double airResisitanceCoeff, double airDenisity, double frontalArea, double accel_max, double decel_max) :
        speed_(0), accel_(0),speed_max_(speed_max), accel_max_(accel_max), decel_max_(decel_max),
        Cd(airResisitanceCoeff), rho(airDenisity), A(frontalArea){}

    void RealSpeedInput(double speed)
    {
        real_speed_ = speed;
    }

    void EngineInput(int engine)
    {
        switch (engine)
        {
            case 1:
                accel_max_ = 0.25;
                decel_max_ = 0.25;
                break;

            case 2:
                accel_max_ = 0.3;
                decel_max_ = 0.2;
                break;

            case 3:
                accel_max_ = 0.35;
                decel_max_ = 0.15;
                break;
            
            default:
                break;
        }
    }

    //加速、减速 
    void UpdateSpeed(double throttle, double brake, double dt)
    {
        //根据油门深度来计算最大速度

        double accel;
        if(throttle == 0 && brake == 0)
        {
            //自然减速
            if(speed_ != 0)
                accel = -0.3/4.0f;
            else
                accel = 0;
        }
        else
        {   
            if(speed_>speed_max_*0.3)
                accel = throttle * accel_max_*throttle_add_ - brake * decel_max_ *throttle_add_* 2.5;
            else
                accel = 2*(throttle * accel_max_ * throttle_add_ - brake * decel_max_ *throttle_add_* 2.5);
        }
        ROS_INFO("accel: %f", accel);
        
        //根据当前油门值和目标速度来计算加速度,考虑空气阻力的影响
        double wind_resistance = CalculateAirResistance(real_speed_);
        double wind_decel = wind_resistance / car_mass;
        accel_ = accel - wind_decel * (speed_ > 0 ? 1 : 0);
        ROS_INFO("wind_decel: %f", wind_decel);

        speed_ += accel_ * dt;
        
        if(speed_ < 0)
        {
            if(throttle == 0 && brake != 0)
            {
                if(speed_ < -0.3)
                    speed_ = -0.3;
            }
            else
                speed_ = 0;
        }
        else 
        {
            if(speed_ > speed_max_)
            {
                speed_ = speed_max_;
            }
        }
    }


    //
    void AddThrottle(bool throttle)
    {
        if(throttle)
        {
            throttle_add_ = 1.1f;
        }
        else
        {
            throttle_add_ = 1.0f;
        }
    }

    //设置档位
    void SetGear(int gear)
    {
        switch (gear)
        {
            case 1:
                speed_max_ = 0.3;
                break;
            
            case 2:
                speed_max_ = 0.6;
                break;
            
            case 3:
                speed_max_ = 0.9;
                break;

            case 4:
                speed_max_ = 1.2;
                break;

            case 5:
                speed_max_ = 1.5;
                break;

            default:
                speed_max_ = 0.3;
                break;
        }
    }

    //获取当前速度
    double GetSpeed() const { return speed_; }

    //获取当前加速度
    double GetAccel() const { return accel_; }
};


class ServerPack
{
public:
    ServerPack(int16_t port,int timeout);
    ~ServerPack();
    void HandleMsg(int fd);
    void Run();
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    geometry_msgs::Twist cmd_vel_;
    nav_msgs::Odometry odom_;
    
    int server_fd_,result_;
    int timeout_;
    float direction_;    //方向，加速度, 减速度
    float throttle_;
    float brake_;
    int dangwei_;
    bool add_throttle_;
    int engine_;
    struct sockaddr_in server_addr;    //地址
    
    void HandleData(char *data);
    void HandleAccel(int throttle, int brake,int direction);
    void OdomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg);

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

