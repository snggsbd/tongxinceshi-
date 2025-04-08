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
#include <signal.h>
#include <ctime>


LegControl legControl;



#include <iostream>
#include <chrono>

// 保护停止信号
void ProtectiveStop(int signum) {
    if (signum == SIGINT) {
        legControl.~LegControl();
    }
}

int main() {
    
    legControl.LegInit();

    double tau[12] = {0.0, 0.0, 0.0};
    double pos_des[12] = {0.0, 0.0, 0.0};
    double vel_des[12] = {0.0, 0.0, 0.0};
    double Kp[12] = {0, 0, 0, 3, 0, 6, 0, 0, 0, 0, 0, 0};
    double Kd[12] = {0.0, 0.0, 1.0, 0.5,0.5,0.5,0.0,0.0,0.0};
    double postion_target[12] = {2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0};
    double tauff[12] = {0.0, 0.0, 0.0};
    const double dt = 5.0 ;  // 5 毫秒转换为秒
    auto start_time = std::chrono::high_resolution_clock::now();


    for (int i = 0; i < 100000; i++) {
        
        signal(SIGINT, ProtectiveStop);

        

        auto current_time = std::chrono::high_resolution_clock::now();
        double time = std::chrono::duration<double>(current_time - start_time).count();  // 计算运行时间（秒）

        // 调用你的函数
        legControl.SlowStandtoPosition(dt, Kp, Kd, time);
        //legControl.


        // 计算已经花费的时间
        auto elapsed = std::chrono::high_resolution_clock::now() - current_time;
        double elapsed_ms = std::chrono::duration<double, std::milli>(elapsed).count();

        // 如果已经花费的时间小于 5 毫秒，则继续等待剩余的时间
        if (elapsed_ms < 5.0) {
            usleep((5.0 - elapsed_ms) * 1000);
        }
    }
}