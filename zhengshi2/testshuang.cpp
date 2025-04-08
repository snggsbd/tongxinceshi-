#include "MotorControl.h"
#include <thread>
#include <atomic>
#include <mutex> // 引入互斥锁
#include <fstream>
#include "LegControl.h"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <mutex> // 引入互斥锁
#include <iostream>
#include <algorithm>
#include <iomanip> 


// void LegControl::RunRecvThread() {
//     std::thread recvThread(&LegControl::RecvDevice1Data, this);
//     recvThread.detach();  // 分离线程，独立执行
// }

// 启动发送命令线程
// void MotorControl::RunSendThread() {
//     std::thread sendThread([this]() {
//         while (1) {
//             SendTauCommand(1, 0);  // 示例命令
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 模拟周期性发送
//         }
//     });
//     sendThread.detach();  // 分离线程，独立执行
// }

int main(){
    // uint32_t canID = 1;
    // uint8_t channel1 = 1;
    // uint8_t channel2 = 2;
   
    // MotorControl motorControl1(canID, channel1);
    // MotorControl motorControl2(canID, channel2);
    // LegControl legControl; // 创建LegControl类的对象

    // legControl.LegInit(); // 通过对象调用LegInit函数
    // // 启动两个线程
    // // legControl.RunRecvThread();
    // motorControl1.RunSendThread();
    // motorControl2.RunSendThread();
    // legControl.RunRecvThread();

    

    // 主线程做其他事情，或者简单地保持主线程运行
    // std::this_thread::sleep_for(std::chrono::minutes(10));  // 让主线程休眠一段时间，模拟程序运行

    LegControl legControl;
    legControl.LegInit(); // 通过对象调用LegInit函数
    std::cout << "sjjd"<<std::endl;
    double tau[12] = {0.0, 0.0, 0.0};
    double pos_des[12] = {0.0, 0.0, 0.0};
    double vel_des[12] = {0.0, 0.0, 0.0};
    double Kp[12] = {10, 10, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0, 0, 0};
    double Kd[12] = {0.0, 0.0, 0.0};
    double tauff[12] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 100000; i++) {
    
        // legControl.send2(pos_des, vel_des, Kp, Kd, tauff); // 通过对象调用


        // legControl.send(tau);
    
    
    }
    
    




    return 0;

}