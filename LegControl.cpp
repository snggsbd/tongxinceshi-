#include "LegControl.h"
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <mutex> // 引入互斥锁

const uint8_t channel1 = 1;
const uint8_t channel2 = 2;
const uint32_t canID1 = 1;
const uint32_t canID2 = 2;
const uint32_t canID3 = 3;

LegControl::LegControl()
    : device1(-1), device2(-1),
      RF(-1, channel1), LF(-1, channel2),
      RH(-1, channel1), LH(-1, channel2),
      timeout(1000000)
{}

LegControl::~LegControl() {
    //失能保护
    RF.disableMotor();
    LF.disableMotor();
    RH.disableMotor();
    LH.disableMotor();



    running = false;
    if (recvDevice1Thread.joinable()) {
        recvDevice1Thread.join();
    }
    if (recvDevice2Thread.joinable()) {
        recvDevice2Thread.join();
    }
    if (device1 >= 0) {
        closeUSBCAN(device1);
    }
    if (device2 >= 0) {
        closeUSBCAN(device2);
    }

}

// void LegControl::ProtectiveStop(int signum) {
//     if (signum == SIGINT) {
//         LegControl();

//     }

// }





void LegControl::LegInit() {

    // data2File.open("data2.csv", std::ios::out);

    if (device1 < 0) {
        device1 = openUSBCAN("/dev/USB2CAN0");
        if (device1 < 0) {
            std::cerr << "Failed to open USB2CAN device 1!" << std::endl;
            // return;
        }
        std::cout << "Device 1 opened successfully." << std::endl;
    }

    if (device2 < 0) {
        device2 = openUSBCAN("/dev/USB2CAN1");
        if (device2 < 0) {
            std::cerr << "Failed to open USB2CAN device 2!" << std::endl;
            // return;
        }
        std::cout << "Device 2 opened successfully." << std::endl;
    }

    // 将设备句柄传递给 MotorControl 对象
    RF = MotorControl(device1, channel1);
    LF = MotorControl(device1, channel2);
    RH = MotorControl(device2, channel1);
    LH = MotorControl(device2, channel2);

    RF.MotorInit();
    LF.MotorInit();
    RH.MotorInit();
    LH.MotorInit();

    running = true;

    // 启动前腿和后腿的接收线程
    recvDevice1Thread = std::thread(&LegControl::RecvDevice1Data, this);
    recvDevice2Thread = std::thread(&LegControl::RecvDevice2Data, this);
}

void LegControl::StopLeg() {
    RF.disableMotor();
    LF.disableMotor();
    RH.disableMotor();
    LH.disableMotor();
}

void LegControl::send(double tauff[12]) {
    RF.SendTauCommand(canID1, tauff[0]);
    RH.SendTauCommand(canID1, tauff[6]);
    LF.SendTauCommand(canID1, tauff[3]);
    LH.SendTauCommand(canID1, tauff[9]);
    usleep(2000);
    RF.SendTauCommand(canID2, tauff[1]);
    RH.SendTauCommand(canID2, tauff[7]);
    LF.SendTauCommand(canID2, tauff[4]);
    LH.SendTauCommand(canID2, tauff[10]);
    usleep(2000);
    RF.SendTauCommand(canID3, tauff[2]);
    LF.SendTauCommand(canID3, tauff[5]);
    RH.SendTauCommand(canID3, tauff[8]);
    LH.SendTauCommand(canID3, tauff[11]);
    usleep(2000);
}




void LegControl::TorqueControl(double pos_des[12]) {
    RF.SendTorqueControlCommand(canID1, pos_des[0], _state.rf_q[0], _state.rf_v[0]);
    RH.SendTorqueControlCommand(canID1, pos_des[0], _state.rh_q[0], _state.rh_v[0]);
    LF.SendTorqueControlCommand(canID1, pos_des[0], _state.lf_q[0], _state.lf_v[0]);
    LH.SendTorqueControlCommand(canID1, pos_des[0], _state.lh_q[0], _state.lh_v[0]);
    // RH.SendTauCommand(canID1, tauff[6]);
    // LF.SendTauCommand(canID1, tauff[3]);
    // LH.SendTauCommand(canID1, tauff[9]);
    usleep(2000);
    RF.SendTorqueControlCommand(canID2, pos_des[1], _state.rf_q[1], _state.rf_v[1]);
    RH.SendTorqueControlCommand(canID2, pos_des[1], _state.rh_q[1], _state.rh_v[1]);
    LF.SendTorqueControlCommand(canID2, pos_des[1], _state.lf_q[1], _state.lf_v[1]);
    LH.SendTorqueControlCommand(canID2, pos_des[1], _state.lh_q[1], _state.lh_v[1]);
    // RF.SendTauCommand(canID2, tauff[1]);
    // RH.SendTauCommand(canID2, tauff[7]);
    // LF.SendTauCommand(canID2, tauff[4]);
    // LH.SendTauCommand(canID2, tauff[10]);
    usleep(2000);
    RF.SendTorqueControlCommand(canID3, pos_des[2], _state.rf_q[2], _state.rf_v[2]);
    RH.SendTorqueControlCommand(canID3, pos_des[2], _state.rh_q[2], _state.rh_v[2]);
    LF.SendTorqueControlCommand(canID1, pos_des[0], _state.rf_q[0], _state.rf_v[0]);
    RH.SendTorqueControlCommand(canID1, pos_des[0], _state.rh_q[0], _state.rh_v[0]);
    LF.SendTorqueControlCommand(canID1, pos_des[0], _state.lf_q[0], _state.lf_v[0]);
    LH.SendTorqueControlCommand(canID1, pos_des[0], _state.lh_q[0], _state.lh_v[0]);
    LH.SendTorqueControlCommand(canID3, pos_des[2], _state.lh_q[2], _state.lh_v[2]);
    // RF.SendTauCommand(canID3, tauff[2]);
    // LF.SendTauCommand(canID3, tauff[5]);
    // RH.SendTauCommand(canID3, tauff[8]);
    // LH.SendTauCommand(canID3, tauff[11]);
    usleep(2000);
}

void LegControl::send2(double pos_des[12], double vel_des[12], double Kp[12] , double Kd[12], double tauff[12]) {
    RF.SendHybridCommand(canID1, pos_des[0], vel_des[0], Kp[0] , Kd[0], tauff[0]); // 每条can通路相邻两次发的时间必须大于333us
    usleep(100);
    RH.SendHybridCommand(canID1, pos_des[6], vel_des[6], Kp[6] , Kd[6], tauff[6]);
    usleep(100);
    LF.SendHybridCommand(canID1, pos_des[3], vel_des[3], Kp[3] , Kd[3], tauff[3]);
    usleep(100);
    LH.SendHybridCommand(canID1, pos_des[9], vel_des[9], Kp[9] , Kd[9], tauff[9]);
    usleep(100);
    RF.SendHybridCommand(canID2, pos_des[1], vel_des[1], Kp[1] , Kd[1], tauff[1]);
    usleep(100);
    RH.SendHybridCommand(canID2, pos_des[7], vel_des[7], Kp[7] , Kd[7], tauff[7]);
    usleep(100);
    LF.SendHybridCommand(canID2, pos_des[4], vel_des[4], Kp[4] , Kd[4], tauff[4]);
    usleep(100);
    LH.SendHybridCommand(canID2, pos_des[10], vel_des[10], Kp[10] , Kd[10], tauff[10]);
    usleep(100);
    RF.SendHybridCommand(canID3, pos_des[2], vel_des[2], Kp[2] , Kd[2], tauff[2]);
    usleep(100);
    RH.SendHybridCommand(canID3, pos_des[8], vel_des[8], Kp[8] , Kd[8], tauff[8]);
    usleep(100);
    LF.SendHybridCommand(canID3, pos_des[5], vel_des[5], Kp[5] , Kd[5], tauff[5]);
    usleep(100);
    LH.SendHybridCommand(canID3, pos_des[11], vel_des[11], Kp[11] , Kd[11], tauff[11]);
    // usleep(100);
}

// 接收第一个模块的反馈数据（前腿）
void LegControl::RecvDevice1Data() {
    while (running) {

        int32_t result = readUSBCAN(device1, &readDevice1Channel, &readDevice1Info, data1, timeout);
        std::cout << "1result: " << result << std::endl;
        
        if (result == 0) {  // 成功接收一帧数据
        auto start = std::chrono::high_resolution_clock::now();
            // 解析数据
            if (readDevice1Channel == 1) { // rf腿的三个电机
                std::cout <<"channel1"        << std::endl;
                for (int i = 0; i < 6; ++i) {
                        std::cout << std::hex << static_cast<int>(data1[i]) << " "; 
                    }
                uint16_t positionRaw = (static_cast<uint16_t>(data1[1]) << 8) | static_cast<uint16_t>(data1[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);
                std::cout << "position: " << position << std::endl;
                uint16_t velocityRaw = (static_cast<uint16_t>(data1[3]) << 4) | (static_cast<uint16_t>(data1[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;
                std::cout << "velocity: " << velocity << std::endl;
                uint16_t torqueRaw = ((static_cast<uint16_t>(data1[4] & 0x0F)) << 8) | static_cast<uint16_t>(data1[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;
                std::cout << "torque: " << torqueNm << std::endl;
                unsigned char id = data1[0];
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.rf_q[motorIndex] = position;
                    _state.rf_v[motorIndex] = velocity;
                    _state.rf_tau[motorIndex] = torqueNm;
                }
            }
            else if (readDevice1Channel == 2) { // lf腿的三个电机
                std::cout <<"channel2"        << std::endl;
                for (int i = 0; i < 6; ++i) {
                        std::cout << std::hex << static_cast<int>(data1[i]) << " ";
                    }

                uint16_t positionRaw = (static_cast<uint16_t>(data1[1]) << 8) | static_cast<uint16_t>(data1[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);
                std::cout << "position: " << position << std::endl;
                uint16_t velocityRaw = (static_cast<uint16_t>(data1[3]) << 4) | (static_cast<uint16_t>(data1[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;
                std::cout << "velocity: " << velocity << std::endl;
                uint16_t torqueRaw = ((static_cast<uint16_t>(data1[4] & 0x0F)) << 8) | static_cast<uint16_t>(data1[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;
                std::cout << "torque: " << torqueNm << std::endl;
                unsigned char id = data1[0];
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.lf_q[motorIndex] = position;
                    _state.lf_v[motorIndex] = velocity;
                    _state.lf_tau[motorIndex] = torqueNm;
                }

            }
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            std::cout << "读取数据总时长: " << elapsed.count() << " 秒" << std::endl;
        }
        else if (result == -1) {
            std::cerr << "模块1 recv timeout!!!" << std::endl;
        }
    }
}

// 接收第二个模块的反馈数据（后腿）
void LegControl::RecvDevice2Data() {
    while (running) {
        int32_t result = readUSBCAN(device2, &readDevice2Channel, &readDevice2Info, data2, timeout);
        std::cout << "262174result: " << result << std::endl;
        if (result == 0) {  // 成功接收一帧数据
            // 解析数据
            if (readDevice2Channel == 1) { // rh腿的三个电机
                uint16_t positionRaw = (static_cast<uint16_t>(data2[1]) << 8) | static_cast<uint16_t>(data2[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);
                std::cout << "position: " << position << std::endl;
                uint16_t velocityRaw = (static_cast<uint16_t>(data2[3]) << 4) | (static_cast<uint16_t>(data2[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;
                std::cout << "velocity: " << velocity << std::endl;
                uint16_t torqueRaw = ((static_cast<uint16_t>(data2[4] & 0x0F)) << 8) | static_cast<uint16_t>(data2[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;
                std::cout << "torque: " << torqueNm << std::endl;       
                unsigned char id = data2[0];
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.rh_q[motorIndex] = position;
                    _state.rh_v[motorIndex] = velocity;
                    _state.rh_tau[motorIndex] = torqueNm;
                } 
            }
            else if (readDevice2Channel == 2) { // lh腿的三个电机
                uint16_t positionRaw = (static_cast<uint16_t>(data2[1]) << 8) | static_cast<uint16_t>(data2[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);
                std::cout << "position: " << position << std::endl;
                uint16_t velocityRaw = (static_cast<uint16_t>(data2[3]) << 4) | (static_cast<uint16_t>(data2[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;
                std::cout << "velocity: " << velocity << std::endl;
                uint16_t torqueRaw = ((static_cast<uint16_t>(data2[4] & 0x0F)) << 8) | static_cast<uint16_t>(data2[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;
                std::cout << "torque: " << torqueNm << std::endl;
                unsigned char id = data2[0];
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.lh_q[motorIndex] = position;
                    _state.lh_v[motorIndex] = velocity;
                    _state.lh_tau[motorIndex] = torqueNm;
                } 
            }
        }
        else if (result == -1) {
            std::cerr << "模块2 recv timeout!!!" << std::endl;
        }
    }
}

LegState LegControl::getState() const {
    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁以安全地读取_state
    return _state;
}


void LegControl::RunRecvThread() {
    std::thread recvThread(&LegControl::RecvDevice1Data, this);
    recvThread.detach();  // 分离线程，独立执行
}

void LegControl::torque_zhengxian(double postion_target[12] , double kp[12] , double kd[12] , double time)
{
    double tauff[12];
    double postion_sin[12];
    postion_sin[0] = postion_target[0] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[1] = postion_target[1] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[2] = postion_target[2] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[3] = postion_target[3] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[4] = postion_target[4] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[5] = postion_target[5] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[6] = postion_target[6] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[7] = postion_target[7] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[8] = postion_target[8] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[9] = postion_target[9] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[10] = postion_target[10] + 0.1 * sin(time * 2 * M_PI);
    postion_sin[11] = postion_target[11] + 0.1 * sin(time * 2 * M_PI);
    tauff[0] = RF.compute_torque(postion_sin[0],_state.rf_q[0],  _state.rf_v[0], kp[0] , kd[0] );
    tauff[1] = RF.compute_torque(postion_sin[1],_state.rf_q[1],  _state.rf_v[1], kp[1] , kd[1] );
    tauff[2] = RF.compute_torque(postion_sin[2],_state.rf_q[2],  _state.rf_v[2], kp[2] , kd[2] );
    tauff[3] = RH.compute_torque(postion_sin[0],_state.rf_q[0],  _state.rf_v[0], kp[0] , kd[0] );
    tauff[4] = RH.compute_torque(postion_sin[1],_state.rf_q[1],  _state.rf_v[1], kp[1] , kd[1] );
    tauff[5] = RH.compute_torque(postion_sin[2],_state.rf_q[2],  _state.rf_v[2], kp[2] , kd[2] );
    tauff[6] = LF.compute_torque(postion_sin[6],_state.lf_q[0],  _state.lf_v[0], kp[6] , kd[6] );
    tauff[7] = LF.compute_torque(postion_sin[7],_state.lf_q[1],  _state.lf_v[1], kp[7] , kd[7] );
    tauff[8] = LF.compute_torque(postion_sin[8],_state.lf_q[2],  _state.lf_v[2], kp[8] , kd[8] );
    tauff[9] = LH.compute_torque(postion_sin[6],_state.lf_q[0],  _state.lf_v[0], kp[6] , kd[6] );
    tauff[10] = LH.compute_torque(postion_sin[7],_state.lf_q[1],  _state.lf_v[1], kp[7] , kd[7] );
    tauff[11] = LH.compute_torque(postion_sin[8],_state.lf_q[2],  _state.lf_v[2], kp[8] , kd[8] );
    
    RF.SendTauCommand(canID1, tauff[0]);
    RH.SendTauCommand(canID1, tauff[6]);
    LF.SendTauCommand(canID1, tauff[3]);
    LH.SendTauCommand(canID1, tauff[9]);
    usleep(2000);
    RF.SendTauCommand(canID2, tauff[1]);
    RH.SendTauCommand(canID2, tauff[7]);
    LF.SendTauCommand(canID2, tauff[4]);
    LH.SendTauCommand(canID2, tauff[10]);
    usleep(2000);
    RF.SendTauCommand(canID3, tauff[2]);
    LF.SendTauCommand(canID3, tauff[5]);
    RH.SendTauCommand(canID3, tauff[8]);
    LH.SendTauCommand(canID3, tauff[11]);
    usleep(2000);
}


void LegControl::torque_zhengxian_dandianji(double postion_target[12] , double kp[12] , double kd[12] , double time)
{
    double tauff[12];
    double postion_sin[12];
    postion_sin[0] = postion_target[0] ;//+ 0.1 * sin(time * 2 * M_PI);
    postion_sin[1] = postion_target[1] ;//+ 0.1 * sin(time * 2 * M_PI);
    postion_sin[2] = postion_target[2] ;//+ 0.2 * sin(time * 2 * M_PI);
    // postion_sin[3] = postion_target[3] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[4] = postion_target[4] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[5] = postion_target[5] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[6] = postion_target[6] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[7] = postion_target[7] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[8] = postion_target[8] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[9] = postion_target[9] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[10] = postion_target[10] + 0.1 * sin(time * 2 * M_PI);
    // postion_sin[11] = postion_target[11] + 0.1 * sin(time * 2 * M_PI);
    // tauff[0] = RF.compute_torque(postion_sin[0],_state.rf_q[0],  _state.rf_v[0], kp[0] , kd[0] );
    tauff[1] = RF.compute_torque(postion_sin[1],_state.rf_q[1],  _state.rf_v[1], kp[1] , kd[1] );
    tauff[2] = RF.compute_torque(postion_sin[2],_state.rf_q[2],  _state.rf_v[2], kp[2] , kd[2] );
    // tauff[3] = RH.compute_torque(postion_sin[0],_state.rf_q[0],  _state.rf_v[0], kp[0] , kd[0] );
    // tauff[4] = RH.compute_torque(postion_sin[1],_state.rf_q[1],  _state.rf_v[1], kp[1] , kd[1] );
    // tauff[5] = RH.compute_torque(postion_sin[2],_state.rf_q[2],  _state.rf_v[2], kp[2] , kd[2] );
    // tauff[6] = LF.compute_torque(postion_sin[6],_state.lf_q[0],  _state.lf_v[0], kp[6] , kd[6] );
    // tauff[7] = LF.compute_torque(postion_sin[7],_state.lf_q[1],  _state.lf_v[1], kp[7] , kd[7] );
    // tauff[8] = LF.compute_torque(postion_sin[8],_state.lf_q[2],  _state.lf_v[2], kp[8] , kd[8] );
    // tauff[9] = LH.compute_torque(postion_sin[6],_state.lf_q[0],  _state.lf_v[0], kp[6] , kd[6] );
    // tauff[10] = LH.compute_torque(postion_sin[7],_state.lf_q[1],  _state.lf_v[1], kp[7] , kd[7] );
    // tauff[11] = LH.compute_torque(postion_sin[8],_state.lf_q[2],  _state.lf_v[2], kp[8] , kd[8] );
    
    // RF.SendTauCommand(canID1, tauff[0]);
    // RH.SendTauCommand(canID1, tauff[6]);
    // LF.SendTauCommand(canID1, tauff[3]);
    // LH.SendTauCommand(canID1, tauff[9]);
    // usleep(2000);
    // RF.SendTauCommand(canID2, tauff[1]);
    // RH.SendTauCommand(canID2, tauff[7]);
    // LF.SendTauCommand(canID2, tauff[4]);
    // LH.SendTauCommand(canID2, tauff[10]);
    usleep(2000);
    if (tauff[2] > 3)
        tauff[2] = 3;
    if (tauff[2] < -3)
        tauff[2] = -3;
    
    RF.SendTauCommand(canID3, tauff[2]);
    // LF.SendTauCommand(canID3, tauff[5]);
    // RH.SendTauCommand(canID3, tauff[8]);
    // LH.SendTauCommand(canID3, tauff[11]);
    // usleep(2000);
}




void LegControl::SlowStandtoPosition(double time_step_, double standing_kp[12] , double standing_kd[12] ,double time ){


    const double dt = time_step_ / 1000.0; // 转换为秒
    int init_joint_pos[12];
    double target_joint_pos[12];
    double joint_positions_[12];
    double joint_velocities_[12];
    double desired_pos[12];
    double error[12];
    double tauff[12];

    

    if (!initialized) {
        // 第一次调用时记录初始位置
        init_joint_pos[0] =  _state.lf_q[0];
        init_joint_pos[1] =  _state.lf_q[1];    
        init_joint_pos[2] =  _state.lf_q[2];
        init_joint_pos[3] =  _state.rf_q[0];
        init_joint_pos[4] =  _state.rf_q[1];
        init_joint_pos[5] =  _state.rf_q[2];
        init_joint_pos[6] =  _state.lh_q[0];
        init_joint_pos[7] =  _state.lh_q[1];
        init_joint_pos[8] =  _state.lh_q[2];
        init_joint_pos[9] =  _state.rh_q[0];
        init_joint_pos[10] = _state.rh_q[1];
        init_joint_pos[11] = _state.rh_q[2];
        initialized = true;
        double elapsed_time = 0.0;
    }
        joint_positions_[0] =  _state.lf_q[0];
        joint_positions_[1] =  _state.lf_q[1];    
        joint_positions_[2] =  _state.lf_q[2];
        joint_positions_[3] =  _state.rf_q[0];
        joint_positions_[4] =  _state.rf_q[1];
        joint_positions_[5] =  _state.rf_q[2];
        joint_positions_[6] =  _state.lh_q[0];
        joint_positions_[7] =  _state.lh_q[1];
        joint_positions_[8] =  _state.lh_q[2];
        joint_positions_[9] =  _state.rh_q[0];
        joint_positions_[10] = _state.rh_q[1];
        joint_positions_[11] = _state.rh_q[2];

        joint_velocities_[0] =  _state.lf_v[0];
        joint_velocities_[1] =  _state.lf_v[1];    
        joint_velocities_[2] =  _state.lf_v[2];
        joint_velocities_[3] =  _state.rf_v[0];
        joint_velocities_[4] =  _state.rf_v[1];
        joint_velocities_[5] =  _state.rf_v[2];
        joint_velocities_[6] =  _state.lh_v[0];
        joint_velocities_[7] =  _state.lh_v[1];
        joint_velocities_[8] =  _state.lh_v[2];
        joint_velocities_[9] =  _state.rh_v[0];
        joint_velocities_[10] = _state.rh_v[1];
        joint_velocities_[11] = _state.rh_v[2];


 
    constexpr double DEFAULT_JOINT_ANGLES[12] = {
        // FL
        -0.1,    // hip
        -0.8,    // thigh
        1.5,   // calf
        
        // FR
        0.1,   // hip  
        -0.8,    // thigh
        -1.5,   // calf
        
        // RL
        0.1,    // hip
        -0.8,    // thigh
        1.5,   // calf
        
        // RR
        -0.1,   // hip
        -0.8,    // thigh
        -1.5    // calf
    };
    
// 等待0.2秒稳定
    if (time < 0.2){
        sleep(0.2);
        return;
    }



    // 拷贝目标位置
    std::copy(DEFAULT_JOINT_ANGLES, DEFAULT_JOINT_ANGLES+12, target_joint_pos);
    double elapsed_time ;
    if (!initialized) {
    //     第一次调用时记录初始位置
        for(int i=0; i<12; ++i){
            init_joint_pos[i] = joint_positions_[i];
        }
        initialized = true;
        elapsed_time = 0.0;
    }
    double transition_time = 6.0;
    elapsed_time += dt;
    
    if (elapsed_time < transition_time) {
        // 计算插值比例 (0~1)
        double ratio = std::min(elapsed_time / transition_time, 1.0);

        // 计算目标位置
        for (int i = 0; i < 12; ++i) {
            std::cout <<  "init_pos["  <<i << "] = " << init_joint_pos[i]<< std::endl;
            desired_pos[i] = init_joint_pos[i] +
                                (target_joint_pos[i] - init_joint_pos[i]) * ratio;
        }
    } else {
        // 过渡完成后保持目标位置
        for (int i = 0; i < 12; ++i) {
        desired_pos[i] = target_joint_pos[i];
        
    
    }
        
    }



        joint_positions_[0] =  _state.lf_q[0];
        joint_positions_[1] =  _state.lf_q[1];    
        joint_positions_[2] =  _state.lf_q[2];
        joint_positions_[3] =  _state.rf_q[0];
        joint_positions_[4] =  _state.rf_q[1];
        joint_positions_[5] =  _state.rf_q[2] ;
        joint_positions_[6] =  _state.lh_q[0];
        joint_positions_[7] =  _state.lh_q[1];
        joint_positions_[8] =  _state.lh_q[2];
        joint_positions_[9] =  _state.rh_q[0];
        joint_positions_[10] = _state.rh_q[1];
        joint_positions_[11] = _state.rh_q[2];

        joint_velocities_[0] =  _state.lf_v[0];
        joint_velocities_[1] =  _state.lf_v[1];    
        joint_velocities_[2] =  _state.lf_v[2];
        joint_velocities_[3] =  _state.rf_v[0];
        joint_velocities_[4] =  _state.rf_v[1];
        joint_velocities_[5] =  _state.rf_v[2];
        joint_velocities_[6] =  _state.lh_v[0];
        joint_velocities_[7] =  _state.lh_v[1];
        joint_velocities_[8] =  _state.lh_v[2];
        joint_velocities_[9] =  _state.rh_v[0];
        joint_velocities_[10] = _state.rh_v[1];
        joint_velocities_[11] = _state.rh_v[2];


    for (int i = 0; i < 12; ++i) {
        // joint_positions_[i] = get_joint_position(i);
        // joint_velocities_[i] = get_joint_velocity(i);

        error[i] = desired_pos[i] - joint_positions_[i];
        tauff[i] = standing_kp[i] * error[i] - standing_kd[i] * joint_velocities_[i];
        if (time <0.2){
            tauff[i] = 0.0;
    }
        // std::cout << "pos[" << i << "] = " << desired_pos[i] << std::endl;
        // 反向关节的扭矩取反
        // if (i == 0 || i == 3 || i == 1 || i == 2 || i == 7 || i == 8) {
        //     tauff[i] *= -1;
        // }

        // 施加扭矩
        
    }
    if (joint_positions_[4] < - 1.5)
    {
        standing_kp[4] = 0.0;
        standing_kd[4] = 1.0;
    }

    
    // tauff[4] = -tauff[4];
    std::string filename ="/home/bane/桌面/renxing/zhengxian_dandianji.csv";
    std::ofstream file("/home/bane/桌面/renxing/zhengxian_dandianji_time_kp10kd2.5.csv", std::ios::app); // 以追加模式打开文件
    file << std::fixed << time << ',' <<desired_pos[4] << ','<<tauff[4]<<','<< joint_positions_[4] << ',' << std::endl; // 写入数据

// 保护程序部分鄂

// 限制扭矩大小
    for (int i = 0; i < 12; ++i) {
        if (tauff[i] > 3)
            tauff[i] = 3;
        if (tauff[i] < -3)
            tauff[i] = -3;
        // std::cout << "tauff[" << i << "] = " << tauff[i] << std::endl;
    }
    
// 限制幅度
    if (joint_positions_[3]>0.15||joint_positions_[3]<-0.05){
        
        standing_kd[3] = 10;
        standing_kp[3] = 0.0;
    }

    if (joint_positions_[4]<-1.1 || joint_positions_[4]>0.5){
        standing_kd[4] = 10;
        standing_kp[4] = 0.0;
    
    }
    // std::cout << "tauff[0] = " << tauff[0] << std::endl;
    // std::cout << "tauff[1] = " << tauff[1] << std::endl;
    // std::cout << "tauff[2] = " << tauff[2] << std::endl;
    // std::cout << "tauff[3] = " << tauff[3] << std::endl;
    // std::cout << "tauff[4] = " << tauff[4] << std::endl;
    // std::cout << "tauff[5] = " << tauff[5] << std::endl;
    // std::cout << "tauff[6] = " << tauff[6] << std::endl;
    // std::cout << "tauff[7] = " << tauff[7] << std::endl;
    // std::cout << "tauff[8] = " << tauff[8] << std::endl;
    // std::cout << "tauff[9] = " << tauff[9] << std::endl;
    // std::cout << "tauff[10] = " << tauff[10] << std::endl;
    // std::cout << "tauff[11] = " << tauff[11] << std::endl;

    

    RF.SendTauCommand(canID1, tauff[3]);
    // RH.SendTauCommand(canID1, tauff[9]);
    // LF.SendTauCommand(canID1, tauff[0]);
    // LH.SendTauCommand(canID1, tauff[6]);

    usleep(500);
    RF.SendTauCommand(canID2, tauff[4]);
    // RH.SendTauCommand(canID2, tauff[10]);
    // LF.SendTauCommand(canID2, tauff[1]);
    // LH.SendTauCommand(canID2, tauff[7]);
    usleep(500);
    RF.SendTauCommand(canID3, tauff[5]);
    // LF.SendTauCommand(canID3, tauff[2]);
    // RH.SendTauCommand(canID3, tauff[11]);
    // LH.SendTauCommand(canID3, tauff[8]);
    usleep(500);
}



void LegControl::SlowStandtoPosition_dan(double time_step_,double standing_kp[12] , double standing_kd[12] ){


    const double dt = time_step_ / 1000.0; // 转换为秒
    int init_joint_pos[12];
    double target_joint_pos[12];
    double joint_positions_[12];
    double joint_velocities_[12];
    double desired_pos[12];
    double error[12];
    double tauff[12];

    if (!initialized) {
        // 第一次调用时记录初始位置
        init_joint_pos[0] =  _state.lf_q[0];
        init_joint_pos[1] =  _state.lf_q[1];    
        init_joint_pos[2] =  _state.lf_q[2];
        // init_joint_pos[3] =  _state.rf_q[0];
        // init_joint_pos[4] =  _state.rf_q[1];
        // init_joint_pos[5] =  _state.rf_q[2];
        // init_joint_pos[6] =  _state.lh_q[0];
        // init_joint_pos[7] =  _state.lh_q[1];
        // init_joint_pos[8] =  _state.lh_q[2];
        // init_joint_pos[9] =  _state.rh_q[0];
        // init_joint_pos[10] = _state.rh_q[1];
        // init_joint_pos[11] = _state.rh_q[2];
        initialized = true;
        double elapsed_time = 0.0;
    }
        joint_positions_[0] =  _state.lf_q[0];
        joint_positions_[1] =  _state.lf_q[1];    
        joint_positions_[2] =  _state.lf_q[2];
        // joint_positions_[3] =  _state.rf_q[0];
        // joint_positions_[4] =  _state.rf_q[1];
        // joint_positions_[5] =  _state.rf_q[2];
        // joint_positions_[6] =  _state.lh_q[0];
        // joint_positions_[7] =  _state.lh_q[1];
        // joint_positions_[8] =  _state.lh_q[2];
        // joint_positions_[9] =  _state.rh_q[0];
        // joint_positions_[10] = _state.rh_q[1];
        // joint_positions_[11] = _state.rh_q[2];

        joint_velocities_[0] =  _state.lf_v[0];
        joint_velocities_[1] =  _state.lf_v[1];    
        joint_velocities_[2] =  _state.lf_v[2];
        // joint_velocities_[3] =  _state.rf_v[0];
        // joint_velocities_[4] =  _state.rf_v[1];
        // joint_velocities_[5] =  _state.rf_v[2];
        // joint_velocities_[6] =  _state.lh_v[0];
        // joint_velocities_[7] =  _state.lh_v[1];
        // joint_velocities_[8] =  _state.lh_v[2];
        // joint_velocities_[9] =  _state.rh_v[0];
        // joint_velocities_[10] = _state.rh_v[1];
        // joint_velocities_[11] = _state.rh_v[2];


 
    constexpr double DEFAULT_JOINT_ANGLES[12] = {
        // FL
        0.1,    // hip
        -0.8,    // thigh
        1.5,   // calf
        
        // FR
        -0.1,   // hip  
        0.8,    // thigh
        -1.5,   // calf
        
        // RL
        0.1,    // hip
        -0.8,    // thigh
        1.5,   // calf
        
        // RR
        -0.1,   // hip
        0.8,    // thigh
        -1.5    // calf
    };
    
    // 拷贝目标位置
    std::copy(DEFAULT_JOINT_ANGLES, DEFAULT_JOINT_ANGLES+12, target_joint_pos);
    
    // if (!initialized) {
    //     第一次调用时记录初始位置
    //     for(int i=0; i<12; ++i){
    //         init_joint_pos[i] = joint_positions_[i];
    //     }
    //     initialized = true;
    double elapsed_time = 0.0;
    // }
    double transition_time = 2.0;
    elapsed_time += dt;
    
    if (elapsed_time < transition_time) {
        // 计算插值比例 (0~1)
        double ratio = std::min(elapsed_time / transition_time, 1.0);

        // 计算目标位置
        for (int i = 0; i < 3; ++i) {
            desired_pos[i] = init_joint_pos[i] +
                                (target_joint_pos[i] - init_joint_pos[i]) * ratio;
        }
    } else {
        // 过渡完成后保持目标位置
        for (int i = 0; i < 3; ++i) {
        desired_pos[i] = target_joint_pos[i];
    }
    }



        joint_positions_[0] =  _state.lf_q[0];
        joint_positions_[1] =  _state.lf_q[1];    
        joint_positions_[2] =  _state.lf_q[2] - 2.306;
        // joint_positions_[3] =  _state.rf_q[0];
        // joint_positions_[4] =  _state.rf_q[1];
        // joint_positions_[5] =  _state.rf_q[2] - 2.306;
        // joint_positions_[6] =  _state.lh_q[0];
        // joint_positions_[7] =  _state.lh_q[1];
        // joint_positions_[8] =  _state.lh_q[2] - 2.306;
        // joint_positions_[9] =  _state.rh_q[0];
        // joint_positions_[10] = _state.rh_q[1];
        // joint_positions_[11] = _state.rh_q[2] - 2.306;

        joint_velocities_[0] =  _state.lf_v[0];
        joint_velocities_[1] =  _state.lf_v[1];    
        joint_velocities_[2] =  _state.lf_v[2];
        // joint_velocities_[3] =  _state.rf_v[0];
        // joint_velocities_[4] =  _state.rf_v[1];
        // joint_velocities_[5] =  _state.rf_v[2];
        // joint_velocities_[6] =  _state.lh_v[0];
        // joint_velocities_[7] =  _state.lh_v[1];
        // joint_velocities_[8] =  _state.lh_v[2];
        // joint_velocities_[9] =  _state.rh_v[0];
        // joint_velocities_[10] = _state.rh_v[1];
        // joint_velocities_[11] = _state.rh_v[2];


    for (int i = 0; i < 3; ++i) {
        // joint_positions_[i] = get_joint_position(i);
        // joint_velocities_[i] = get_joint_velocity(i);

        error[i] = desired_pos[i] - joint_positions_[i];
        tauff[i] = standing_kp[i] * error[i] - standing_kd[i] * joint_velocities_[i];

        // 反向关节的扭矩取反
        if (i == 0 || i == 3 || i == 1 || i == 2 || i == 7 || i == 8) {
            tauff[i] *= -1;
        }

        // 施加扭矩
        
    }
    // RF.SendTauCommand(canID1, tauff[3]);
    // RH.SendTauCommand(canID1, tauff[9]);
    LF.SendTauCommand(canID1, tauff[0]);
    // LH.SendTauCommand(canID1, tauff[6]);
    usleep(2000);
    // RF.SendTauCommand(canID2, tauff[4]);
    // RH.SendTauCommand(canID2, tauff[10]);
    LF.SendTauCommand(canID2, tauff[1]);
    // LH.SendTauCommand(canID2, tauff[7]);
    usleep(2000);
    // RF.SendTauCommand(canID3, tauff[5]);
    LF.SendTauCommand(canID3, tauff[2]);
    // RH.SendTauCommand(canID3, tauff[11]);
    // LH.SendTauCommand(canID3, tauff[8]);
    usleep(2000);
}


