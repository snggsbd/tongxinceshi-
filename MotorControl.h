#pragma once

#include "usb_can.h"


class MotorControl {
public:
    MotorControl(int32_t device, uint8_t channel);  // 直接传入设备句柄
    ~MotorControl();

    bool MotorInit();
    
    void ComputeCommandFromTau(uint32_t canID, double tau);
    void ComputeHybridCommand(uint32_t canID, double pos, double vel, double kp, double kd, double tau);

    void SendTauCommand(uint32_t canID, double tau);
    void SendHybridCommand(uint32_t canID, double pos, double vel, double kp, double kd, double tau);
    void SendTorqueControlCommand(uint32_t canID, double target_pos , double current_pos, double velocity);

    double compute_torque(double desired_positions,double current_positions,double current_velocities,double kp,double kd);//力矩转换函数


    void sendTorqueToServer(uint32_t canID, float set_torque); 
       
    void RunSendThread();


    void enableMotor();
    void disableMotor();

private:

    int32_t device;  // 设备句柄
    uint8_t _channel;
    FrameInfo sendInfo,enableInfo,disableInfo; 
    uint8_t data[8];   
};