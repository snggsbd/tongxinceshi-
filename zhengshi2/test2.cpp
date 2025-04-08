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

int main()
{   
    uint32_t can_ID = 0x1;
    double tau = 0.0;
    MotorControl motorControl(can_ID,tau);
    motorControl.SendTauCommand( can_ID , tau);

    return 0;


}





