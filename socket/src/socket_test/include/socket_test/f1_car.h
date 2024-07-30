#include <iostream>

class F1Car
{
private:
    double speed_;  //当前速度
    double accel_;  //当前加速度
    double accel_max_;  //最大加速度
    double accel_min_;  //最小加速度
    double dt_; //时间间隔

public:
    F1Car(double accel_max, double accel_min, double dt) :
        speed_(0), accel_(0), accel_max_(accel_max), accel_min_(accel_min), dt_(dt) {}

    void UpdateSpeed(double throttle)
    {
        //根据油门深度来计算加速度
        accel_ = throttle * accel_max_ + (1 - throttle) * accel_min_;

        //根据加速度来计算速度
        speed_ += accel_ * dt_;

        if(speed_ < 0)
        {
            speed_ = 0;
        }
    }

    //获取当前速度
    double GetSpeed() const { return speed_; }

    //获取当前加速度
    double GetAccel() const { return accel_; }
};

