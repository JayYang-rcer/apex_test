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

//1.电池
//2.限速
//3.引擎映射
class F1Car
{
private:
    double speed_=0;  //当前速度
    double accel_;  //当前加速度
    double speed_max_;  //最大速度
    double accel_max_,l_accel_max_;  //最大加速度
    double decel_max_,l_decel_max_;  //最大减速度
    double Cd; //空气阻力系数
    double A;  //横截面积
    double rho;  //空气密度
    double car_mass=3;  //车重
    double real_speed_;
    double throttle_add_ = 1.0f;
    double engine_map_gap_;
    int engine_select_ = 2;
    double accel_add_ = 1.0f;

    double CalculateAirResistance(double speed)
    {
//        return 0.5 * Cd * A * rho * speed * speed;
         return 0.5 * Cd * A * rho * speed_ * speed_;
    }

public:
    F1Car(int engine_select, double airResisitanceCoeff, double airDenisity, double frontalArea, double accel_max, double decel_max,
          double engine_map_gap):
          speed_(0), accel_(0), l_accel_max_(accel_max), l_decel_max_(decel_max),Cd(airResisitanceCoeff),
          rho(airDenisity), A(frontalArea), engine_map_gap_(engine_map_gap), engine_select_(engine_select)
          {
            switch (engine_select_)
            {
                case 1:
                    speed_max_ = 1.5;
                    accel_add_ = 1.0;
                    break;

                case 2:
                    speed_max_ = 1.75;
                    accel_add_ = 1.1;
                    break;

                case 3:
                    speed_max_ = 2.0;
                    accel_add_ = 1.15;
                    break;

                default: break;
            }
          }

    void RealSpeedInput(double speed)
    {
        real_speed_ = speed;
    }

    void EngineInput(int engine)
    {
        static int last_engine = 0;
        if(engine != last_engine)
        {
            switch (engine)
            {
                case 1:
                    accel_max_ = l_accel_max_ * (1-engine_map_gap_);
                    decel_max_ = l_decel_max_ / (1-engine_map_gap_);
                    break;

                case 2:
                    accel_max_ = l_accel_max_ * 1;
                    decel_max_ = l_decel_max_ / 1;
                    break;

                case 3:
                    accel_max_ = l_accel_max_ / (1+engine_map_gap_);
                    decel_max_ = l_decel_max_ * (1+engine_map_gap_);
                    break;

                default: break;
            }
        }
        last_engine = engine;
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
                accel = -accel_max_/4.0f;
            else
                accel = 0;
        }
        else if(brake != 0)
        {
            accel = -brake * decel_max_ * throttle_add_ * 2;
        }
        else
        {
            if (speed_ > speed_max_ * 0.3)
                accel = throttle * accel_max_ * throttle_add_ * accel_add_;
            else
                accel = 1.5 * (throttle * accel_max_ * throttle_add_ * accel_add_);
        }

        
        //根据当前油门值和目标速度来计算加速度,考虑空气阻力的影响
        double wind_resistance = CalculateAirResistance(real_speed_);
        double wind_decel = wind_resistance / car_mass;
        accel_ = accel - wind_decel * (speed_ > 0 ? 1 : 0);
        ROS_INFO("wind_decel: %f", wind_decel);
        ROS_INFO("accel: %f", accel_);

        speed_ += accel_ * dt;
        
        if(speed_ < 0)
        {
            if(throttle == 0 && brake != 0)
            {
                //倒车限速
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


    //设置电池
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

