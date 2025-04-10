#pragma once

#include "MotorControl.h"
#include <thread>
#include <atomic>
#include <mutex> // 引入互斥锁
#include <fstream>

struct LegState
{
    double rf_q[3], lf_q[3], rh_q[3], lh_q[3];
    double rf_v[3], lf_v[3], rh_v[3], lh_v[3];
    double rf_tau[3], lf_tau[3], rh_tau[3], lh_tau[3];
};

class LegControl {
public:
    LegControl();
    ~LegControl();

   
    
    void LegInit();
    void StopLeg();

    void send(double tauff[12]);
    void send2(double pos_des[12], double vel_des[12], double Kp[12] , double Kd[12], double tauff[12]);
    void TorqueControl(double target_pos[12]);
    void SlowStandtoPosition(double time_step_,double standing_kp_[12] , double standing_kd_[12] ,double time, bool init);


    void RecvDevice1Data();
    void RecvDevice2Data();
    void RunRecvThread();
    double stiffness[12];
    void SlowStandtoPosition_dan(double time_step_,double standing_kp[12] , double standing_kd[12] );
    void torque_zhengxian(double postion_target[12] , double kp[12] , double kd[12] , double time); 
    void torque_zhengxian_dandianji(double postion_target[12] , double kp[12] , double kd[12] , double time); 
    void ProtectiveStop(int signum);



    LegState getState() const;


private:
    int32_t device1;  // 第一个设备的句柄
    int32_t device2;  // 第二个设备的句柄
    bool initialized = false;


    uint8_t readDevice1Channel;
    uint8_t readDevice2Channel;
    FrameInfo readDevice1Info;
    FrameInfo readDevice2Info;
    uint8_t data1[6], data2[6];

    std::thread recvDevice1Thread; // 前腿接收线程
    std::thread recvDevice2Thread; // 后腿接收线程
    std::atomic<bool> running;

    mutable std::mutex stateMutex; // 互斥锁，mutable 允许在 const 函数中锁定

    MotorControl RF, LF, RH, LH;
    LegState _state;
    int32_t timeout;

    // std::ofstream data2File;
};
