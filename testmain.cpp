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


int main(){

    LegControl legControl; // 创建LegControl类的对象

    legControl.LegInit(); // 通过对象调用LegInit函数
    

    legControl.RecvDevice1Data(); // 启动接收CAN数据线程







    return 0;

}
