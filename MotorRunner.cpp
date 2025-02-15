#include <iostream>
#include <stdint.h>
#include "usb_can.h"
#include <thread>
#include <chrono>
// #include "change.h"

int main() {
    // 打开 USB2CAN0 模块
    int32_t dev = openUSBCAN("/dev/USB2CAN0");
    if (dev < 0) {
        std::cerr << "Failed to open USB2CAN device" << std::endl;
        return -1;
    }
    uint8_t channel = 2;//设置通道为2

    // 创建并设置 CAN 帧信息
    FrameInfo info;
    // info.canID = 0x3;            // 设置 CAN ID 为 0x02
    info.frameType = STANDARD;     // 设置为标准帧
    info.dataLength = 8;           // 设置数据长度为 8 字节

    
    // uint8_t data[8];

    // 设置要发送的数据，指令为 0x08 0x00 0x00 0x01 0xFF 0xFF 0xFF 0xFC
    uint8_t data[8] = { 0xff ,0xFF, 0xFF, 0xFF, 0xFf, 0xff, 0xff, 0xfc};
    uint8_t data2[8] ;


    auto start = std::chrono::high_resolution_clock::now();//计时


    // 发送 CAN 帧到 CAN1
    info.canID = 0x1;            // 设置 CAN ID 为 0x02
    sendUSBCAN(dev, 1, &info, data);
    
    readUSBCAN(dev, &channel, &info, data2, 2000);
    for (int i = 0; i < 6; ++i) {
        std::cout << std::hex << static_cast<int>(data2[i]) << " ";
    }
    std::cout << std::endl;


   


    closeUSBCAN(dev);


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    
    // 关闭 USB2CAN0 模块
    // closeUSBCAN(dev);
    // 输出运行时长
    std::cout << "运行时长: " << elapsed.count() << " 秒" << std::endl;

    return 0;
}
