#include "DriverConnection.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

// Global variables
int nPortID1 = 0;  // Port for Motor 1
int nPortID2 = 0;  // Port for Motor 2
unsigned char iSlaveNo1 = 0;  // Motor 1
unsigned char iSlaveNo2 = 1;  // Motor 2
// int nPortID3 = 2;  // Port for Motor 3
// int nPortID4 = 3;  // Port for Motor 4
// int nPortID5 = 4;  // Port for Motor 5
// int nPortID6 = 5;  // Port for Motor 6
// unsigned char iSlaveNo3 = 2;  // Motor 3
// unsigned char iSlaveNo4 = 3;  // Motor 4
// unsigned char iSlaveNo5 = 4;  // Motor 5
// unsigned char iSlaveNo6 = 5;  // Motor 6

const unsigned int base_velocity = 5000;  // Base velocity for all movements

std::vector<int> recorded_positions_m1;
std::vector<int> recorded_positions_m2;
// std::vector<int> recorded_positions_m3;
// std::vector<int> recorded_positions_m4;
// std::vector<int> recorded_positions_m5;
// std::vector<int> recorded_positions_m6;
std::mutex rec_mtx;
std::atomic<bool> recording{false};
std::thread rec_thread;

// Function declarations
void initializeSystem();
void showCommands();
bool handleServoOn();
bool handleServoOff();
bool handleHome();
void handleRecord();
void handleStop();
void handleClear();
void handleGetPos();
void handleGo();
void handleMovePos();
void handleGoPos();
void handlePosTable();
void handlePrintTable();
void recordingThread();
void cleanup();

int main()
{
    initializeSystem();
    showCommands();
    
    std::string line;
    while (true)
    {
        if (!std::getline(std::cin, line)) break;
        std::string cmd = line;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c){ return std::tolower(c); });
        
        if (cmd == "q") break;
        if (cmd.empty()) continue;

        if (cmd == "on") {
            handleServoOn();
        }
        else if (cmd == "off") {
            handleServoOff();
        }
        else if (cmd == "home") {
            handleHome();
        }
        else if (cmd == "record") {
            handleRecord();
        }
        else if (cmd == "stop") {
            handleStop();
        }
        else if (cmd == "clear") {
            handleClear();
        }
        else if (cmd == "getpos") {
            handleGetPos();
        }
        else if (cmd == "go") {
            handleGo();
        }
        else if (cmd == "movepos" || cmd == "move pos") {
            handleMovePos();
        }
        else if (cmd == "go pos" || cmd == "gopos") {
            handleGoPos();
        }
        else if (cmd == "postable") {
            handlePosTable();
        }
        else if (cmd == "print table" || cmd == "printtable") {
            handlePrintTable();
        }
        else {
            std::cout << "Unknown command. Use 'on', 'off', 'home', 'record', 'stop', 'clear', 'getpos', 'go', 'movepos', 'go pos', 'postable', 'print table', or 'q'." << std::endl;
        }
    }
    
    cleanup();
    return 0;
}

// Function implementations
void initializeSystem() {
    const wchar_t* sPort = L"ttyUSB0";
    unsigned int dwBaudRate = 115200;
    
    // Connect and show drive info
    // For now, both motors use the same port (can be changed later)
    Driver_Connection(sPort, dwBaudRate, nPortID1, iSlaveNo1);
    Driver_Connection(sPort, dwBaudRate, nPortID1, iSlaveNo2);
    // Driver_Connection(sPort, dwBaudRate, nPortID3, iSlaveNo3);  // Motor 3
    // Driver_Connection(sPort, dwBaudRate, nPortID4, iSlaveNo4);  // Motor 4
    // Driver_Connection(sPort, dwBaudRate, nPortID5, iSlaveNo5);  // Motor 5
    // Driver_Connection(sPort, dwBaudRate, nPortID6, iSlaveNo6);  // Motor 6
}

void showCommands() {
    std::cout << "=== DUAL AXIS SERVO CONTROL ===" << std::endl;
    // std::cout << "=== 6-AXIS SERVO CONTROL ===" << std::endl;  // Uncomment for 6-axis
    std::cout << "on: Servo on (both motors)" << std::endl;
    // std::cout << "on: Servo on (all 6 motors)" << std::endl;  // Uncomment for 6-axis
    std::cout << "off: Servo off (both motors)" << std::endl;
    // std::cout << "off: Servo off (all 6 motors)" << std::endl;  // Uncomment for 6-axis
    std::cout << "home: Homing both motors simultaneously (Only work in servo on)" << std::endl;
    // std::cout << "home: Homing all 6 motors simultaneously (Only work in servo on)" << std::endl;  // Uncomment for 6-axis
    std::cout << "record: Start recording both motors (Only work in servo off)" << std::endl;
    // std::cout << "record: Start recording all 6 motors (Only work in servo off)" << std::endl;  // Uncomment for 6-axis
    std::cout << "stop: Stop recording" << std::endl;
    std::cout << "clear: Clear buffer" << std::endl;
    std::cout << "getpos: Print current positions of both motors" << std::endl;
    std::cout << "go: Go with positions in buffer for both motors (Only work in servo on)" << std::endl;
    // std::cout << "go: Go with positions in buffer for all 6 motors (Only work in servo on)" << std::endl;  // Uncomment for 6-axis
    std::cout << "movepos: Move both motors to specified positions (Only work in servo on)" << std::endl;
    // std::cout << "movepos: Move all 6 motors to specified positions (Only work in servo on)" << std::endl;  // Uncomment for 6-axis
    std::cout << "go pos: Go through items saved in position table for both motors (Only work in servo on)" << std::endl;
    // std::cout << "go pos: Go through items saved in position table for all 6 motors (Only work in servo on)" << std::endl;  // Uncomment for 6-axis
    std::cout << "postable: Save record to position table" << std::endl;
    std::cout << "print table: Print position table" << std::endl;
    std::cout << "q: Quit" << std::endl;
}

bool handleServoOn() {
    bool success1 = ServoOn(nPortID1, iSlaveNo1);
    bool success2 = ServoOn(nPortID2, iSlaveNo2);
    // bool success3 = ServoOn(nPortID3, iSlaveNo3);  // Motor 3
    // bool success4 = ServoOn(nPortID4, iSlaveNo4);  // Motor 4
    // bool success5 = ServoOn(nPortID5, iSlaveNo5);  // Motor 5
    // bool success6 = ServoOn(nPortID6, iSlaveNo6);  // Motor 6
    
    if (success1 && success2) {
    // if (success1 && success2 && success3 && success4 && success5 && success6) {  // For 6-axis
        printf("Both servos ON successfully.\n");
        // printf("All 6 servos ON successfully.\n");  // For 6-axis
        return true;
    } else {
        printf("Servo ON failed - Motor1: %s, Motor2: %s\n", 
               success1 ? "OK" : "FAIL", 
               success2 ? "OK" : "FAIL");
        // printf("Servo ON failed - M1: %s, M2: %s, M3: %s, M4: %s, M5: %s, M6: %s\n",  // For 6-axis
        //        success1 ? "OK" : "FAIL", success2 ? "OK" : "FAIL", success3 ? "OK" : "FAIL",
        //        success4 ? "OK" : "FAIL", success5 ? "OK" : "FAIL", success6 ? "OK" : "FAIL");
        return false;
    }
}

bool handleServoOff() {
    bool success1 = ServoOff(nPortID1, iSlaveNo1);
    bool success2 = ServoOff(nPortID2, iSlaveNo2);
    // bool success3 = ServoOff(nPortID3, iSlaveNo3);  // Motor 3
    // bool success4 = ServoOff(nPortID4, iSlaveNo4);  // Motor 4
    // bool success5 = ServoOff(nPortID5, iSlaveNo5);  // Motor 5
    // bool success6 = ServoOff(nPortID6, iSlaveNo6);  // Motor 6
    
    if (success1 && success2) {
    // if (success1 && success2 && success3 && success4 && success5 && success6) {  // For 6-axis
        printf("Both servos OFF successfully.\n");
        // printf("All 6 servos OFF successfully.\n");  // For 6-axis
        return true;
    } else {
        printf("Servo OFF failed - Motor1: %s, Motor2: %s\n", 
               success1 ? "OK" : "FAIL", 
               success2 ? "OK" : "FAIL");
        // printf("Servo OFF failed - M1: %s, M2: %s, M3: %s, M4: %s, M5: %s, M6: %s\n",  // For 6-axis
        //        success1 ? "OK" : "FAIL", success2 ? "OK" : "FAIL", success3 ? "OK" : "FAIL",
        //        success4 ? "OK" : "FAIL", success5 ? "OK" : "FAIL", success6 ? "OK" : "FAIL");
        return false;
    }
}

bool handleHome() {
    // Check servo status for both motors
    EZISERVO_AXISSTATUS st1, st2;
    // EZISERVO_AXISSTATUS st3, st4, st5, st6;  // For 6-axis
    
    if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) != FMM_OK ||
        FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) != FMM_OK)
        // FAS_GetAxisStatus(nPortID3, iSlaveNo3, &(st3.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID4, iSlaveNo4, &(st4.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID5, iSlaveNo5, &(st5.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID6, iSlaveNo6, &(st6.dwValue)) != FMM_OK)    // For 6-axis
    {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return false;
    }
    
    if (!st1.FFLAG_SERVOON || !st2.FFLAG_SERVOON)
        // !st3.FFLAG_SERVOON || !st4.FFLAG_SERVOON || !st5.FFLAG_SERVOON || !st6.FFLAG_SERVOON)  // For 6-axis
    {
        std::cout << "One or both servos are OFF. Turn them ON before homing." << std::endl;
        // std::cout << "One or more servos are OFF. Turn them ON before homing." << std::endl;  // For 6-axis
        return false;
    }

    // Send homing commands to both motors (non-blocking approach)
    printf("Sending homing commands to both motors...\n");
    // printf("Sending homing commands to all 6 motors...\n");  // For 6-axis
    
    if (FAS_MoveSingleAxisAbsPos(nPortID1, iSlaveNo1, 0, base_velocity) != FMM_OK)
    {
        printf("Motor 1 homing command failed.\n");
        return false;
    }
    
    if (FAS_MoveSingleAxisAbsPos(nPortID2, iSlaveNo2, 0, base_velocity) != FMM_OK)
    {
        printf("Motor 2 homing command failed.\n");
        return false;
    }
    
    // if (FAS_MoveSingleAxisAbsPos(nPortID3, iSlaveNo3, 0, base_velocity) != FMM_OK)  // For 6-axis
    // {
    //     printf("Motor 3 homing command failed.\n");
    //     return false;
    // }
    
    // if (FAS_MoveSingleAxisAbsPos(nPortID4, iSlaveNo4, 0, base_velocity) != FMM_OK)  // For 6-axis
    // {
    //     printf("Motor 4 homing command failed.\n");
    //     return false;
    // }
    
    // if (FAS_MoveSingleAxisAbsPos(nPortID5, iSlaveNo5, 0, base_velocity) != FMM_OK)  // For 6-axis
    // {
    //     printf("Motor 5 homing command failed.\n");
    //     return false;
    // }
    
    // if (FAS_MoveSingleAxisAbsPos(nPortID6, iSlaveNo6, 0, base_velocity) != FMM_OK)  // For 6-axis
    // {
    //     printf("Motor 6 homing command failed.\n");
    //     return false;
    // }
    
    // Wait for both motors to finish
    bool motor1_done = false, motor2_done = false;
    // bool motor3_done = false, motor4_done = false, motor5_done = false, motor6_done = false;  // For 6-axis
    
    printf("Waiting for both motors to complete homing...\n");
    // printf("Waiting for all 6 motors to complete homing...\n");  // For 6-axis
    
    while (!motor1_done || !motor2_done)
        // !motor3_done || !motor4_done || !motor5_done || !motor6_done)  // For 6-axis
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        if (!motor1_done)
        {
            if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) == FMM_OK)
            {
                motor1_done = !st1.FFLAG_MOTIONING;
                if (motor1_done) printf("Motor 1 homing complete.\n");
            }
        }
        
        if (!motor2_done)
        {
            if (FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) == FMM_OK)
            {
                motor2_done = !st2.FFLAG_MOTIONING;
                if (motor2_done) printf("Motor 2 homing complete.\n");
            }
        }
        
        // if (!motor3_done)  // For 6-axis
        // {
        //     if (FAS_GetAxisStatus(nPortID3, iSlaveNo3, &(st3.dwValue)) == FMM_OK)
        //     {
        //         motor3_done = !st3.FFLAG_MOTIONING;
        //         if (motor3_done) printf("Motor 3 homing complete.\n");
        //     }
        // }
        
        // if (!motor4_done)  // For 6-axis
        // {
        //     if (FAS_GetAxisStatus(nPortID4, iSlaveNo4, &(st4.dwValue)) == FMM_OK)
        //     {
        //         motor4_done = !st4.FFLAG_MOTIONING;
        //         if (motor4_done) printf("Motor 4 homing complete.\n");
        //     }
        // }
        
        // if (!motor5_done)  // For 6-axis
        // {
        //     if (FAS_GetAxisStatus(nPortID5, iSlaveNo5, &(st5.dwValue)) == FMM_OK)
        //     {
        //         motor5_done = !st5.FFLAG_MOTIONING;
        //         if (motor5_done) printf("Motor 5 homing complete.\n");
        //     }
        // }
        
        // if (!motor6_done)  // For 6-axis
        // {
        //     if (FAS_GetAxisStatus(nPortID6, iSlaveNo6, &(st6.dwValue)) == FMM_OK)
        //     {
        //         motor6_done = !st6.FFLAG_MOTIONING;
        //         if (motor6_done) printf("Motor 6 homing complete.\n");
        //     }
        // }
    }
    
    // Get final positions
    int actPos1 = 0, actPos2 = 0;
    // int actPos3 = 0, actPos4 = 0, actPos5 = 0, actPos6 = 0;  // For 6-axis
    
    if (FAS_GetActualPos(nPortID1, iSlaveNo1, &actPos1) == FMM_OK &&
        FAS_GetActualPos(nPortID2, iSlaveNo2, &actPos2) == FMM_OK)
        // FAS_GetActualPos(nPortID3, iSlaveNo3, &actPos3) == FMM_OK &&  // For 6-axis
        // FAS_GetActualPos(nPortID4, iSlaveNo4, &actPos4) == FMM_OK &&  // For 6-axis
        // FAS_GetActualPos(nPortID5, iSlaveNo5, &actPos5) == FMM_OK &&  // For 6-axis
        // FAS_GetActualPos(nPortID6, iSlaveNo6, &actPos6) == FMM_OK)    // For 6-axis
    {
        std::cout << "Both motors homing complete. Motor1 = " << actPos1 
                  << ", Motor2 = " << actPos2 << std::endl;
        // std::cout << "All motors homing complete. M1=" << actPos1 << ", M2=" << actPos2  // For 6-axis
        //           << ", M3=" << actPos3 << ", M4=" << actPos4 << ", M5=" << actPos5 
        //           << ", M6=" << actPos6 << std::endl;
    }
    else
    {
        std::cout << "Both motors homing complete." << std::endl;
        // std::cout << "All 6 motors homing complete." << std::endl;  // For 6-axis
    }
    return true;
}

void recordingThread() {
    int lastPos1 = 0, lastPos2 = 0;
    // int lastPos3 = 0, lastPos4 = 0, lastPos5 = 0, lastPos6 = 0;  // For 6-axis
    bool hasLast1 = false, hasLast2 = false;
    // bool hasLast3 = false, hasLast4 = false, hasLast5 = false, hasLast6 = false;  // For 6-axis
    
    while (recording.load()) {
        int pos1 = 0, pos2 = 0;
        // int pos3 = 0, pos4 = 0, pos5 = 0, pos6 = 0;  // For 6-axis
        
        // Read positions for both motors
        bool readOk1 = (FAS_GetActualPos(nPortID1, iSlaveNo1, &pos1) == FMM_OK);
        bool readOk2 = (FAS_GetActualPos(nPortID2, iSlaveNo2, &pos2) == FMM_OK);
        // bool readOk3 = (FAS_GetActualPos(nPortID3, iSlaveNo3, &pos3) == FMM_OK);  // For 6-axis
        // bool readOk4 = (FAS_GetActualPos(nPortID4, iSlaveNo4, &pos4) == FMM_OK);  // For 6-axis
        // bool readOk5 = (FAS_GetActualPos(nPortID5, iSlaveNo5, &pos5) == FMM_OK);  // For 6-axis
        // bool readOk6 = (FAS_GetActualPos(nPortID6, iSlaveNo6, &pos6) == FMM_OK);  // For 6-axis
        
        if (readOk1 && readOk2) {
        // if (readOk1 && readOk2 && readOk3 && readOk4 && readOk5 && readOk6) {  // For 6-axis
            // Check if any motor position changed
            if (!hasLast1 || !hasLast2 || pos1 != lastPos1 || pos2 != lastPos2) {
            // if (!hasLast1 || !hasLast2 || !hasLast3 || !hasLast4 || !hasLast5 || !hasLast6 ||  // For 6-axis
            //     pos1 != lastPos1 || pos2 != lastPos2 || pos3 != lastPos3 || pos4 != lastPos4 || pos5 != lastPos5 || pos6 != lastPos6) {
                // Record both motors simultaneously
                {
                    std::lock_guard<std::mutex> lk(rec_mtx);
                    recorded_positions_m1.push_back(pos1);
                    recorded_positions_m2.push_back(pos2);
                    // recorded_positions_m3.push_back(pos3);  // For 6-axis
                    // recorded_positions_m4.push_back(pos4);  // For 6-axis
                    // recorded_positions_m5.push_back(pos5);  // For 6-axis
                    // recorded_positions_m6.push_back(pos6);  // For 6-axis
                }
                
                lastPos1 = pos1;
                lastPos2 = pos2;
                // lastPos3 = pos3; lastPos4 = pos4; lastPos5 = pos5; lastPos6 = pos6;  // For 6-axis
                hasLast1 = true;
                hasLast2 = true;
                // hasLast3 = true; hasLast4 = true; hasLast5 = true; hasLast6 = true;  // For 6-axis
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void handleRecord() {
    if (recording.load()) {
        std::cout << "Already recording." << std::endl;
        return;
    }
    recording.store(true);
    rec_thread = std::thread(recordingThread);
    std::cout << "Recording started for both motors (capturing positions only). Type 'stop' to end." << std::endl;
}

void handleStop() {
    if (!recording.load()) {
        std::cout << "Not recording." << std::endl;
        return;
    }
    recording.store(false);
    if (rec_thread.joinable()) rec_thread.join();
    std::lock_guard<std::mutex> lk(rec_mtx);
    std::cout << "Recording stopped." << std::endl;
    std::cout << "Motor 1 samples: " << recorded_positions_m1.size() << std::endl;
    std::cout << "Motor 2 samples: " << recorded_positions_m2.size() << std::endl;
    // std::cout << "Motor 3 samples: " << recorded_positions_m3.size() << std::endl;  // For 6-axis
    // std::cout << "Motor 4 samples: " << recorded_positions_m4.size() << std::endl;  // For 6-axis
    // std::cout << "Motor 5 samples: " << recorded_positions_m5.size() << std::endl;  // For 6-axis
    // std::cout << "Motor 6 samples: " << recorded_positions_m6.size() << std::endl;  // For 6-axis
    
    if (!recorded_positions_m1.empty()) {
        std::cout << "Motor 1 Positions: [";
        for (size_t i = 0; i < recorded_positions_m1.size(); ++i) {
            std::cout << recorded_positions_m1[i];
            if (i + 1 < recorded_positions_m1.size()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    if (!recorded_positions_m2.empty()) {
        std::cout << "Motor 2 Positions: [";
        for (size_t i = 0; i < recorded_positions_m2.size(); ++i) {
            std::cout << recorded_positions_m2[i];
            if (i + 1 < recorded_positions_m2.size()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    // if (!recorded_positions_m3.empty()) {  // For 6-axis
    //     std::cout << "Motor 3 Positions: [";
    //     for (size_t i = 0; i < recorded_positions_m3.size(); ++i) {
    //         std::cout << recorded_positions_m3[i];
    //         if (i + 1 < recorded_positions_m3.size()) std::cout << ", ";
    //     }
    //     std::cout << "]" << std::endl;
    // }
    
    // if (!recorded_positions_m4.empty()) {  // For 6-axis
    //     std::cout << "Motor 4 Positions: [";
    //     for (size_t i = 0; i < recorded_positions_m4.size(); ++i) {
    //         std::cout << recorded_positions_m4[i];
    //         if (i + 1 < recorded_positions_m4.size()) std::cout << ", ";
    //     }
    //     std::cout << "]" << std::endl;
    // }
    
    // if (!recorded_positions_m5.empty()) {  // For 6-axis
    //     std::cout << "Motor 5 Positions: [";
    //     for (size_t i = 0; i < recorded_positions_m5.size(); ++i) {
    //         std::cout << recorded_positions_m5[i];
    //         if (i + 1 < recorded_positions_m5.size()) std::cout << ", ";
    //     }
    //     std::cout << "]" << std::endl;
    // }
    
    // if (!recorded_positions_m6.empty()) {  // For 6-axis
    //     std::cout << "Motor 6 Positions: [";
    //     for (size_t i = 0; i < recorded_positions_m6.size(); ++i) {
    //         std::cout << recorded_positions_m6[i];
    //         if (i + 1 < recorded_positions_m6.size()) std::cout << ", ";
    //     }
    //     std::cout << "]" << std::endl;
    // }
}

void handleClear() {
    std::lock_guard<std::mutex> lk(rec_mtx);
    recorded_positions_m1.clear();
    recorded_positions_m2.clear();
    // recorded_positions_m3.clear();  // For 6-axis
    // recorded_positions_m4.clear();  // For 6-axis
    // recorded_positions_m5.clear();  // For 6-axis
    // recorded_positions_m6.clear();  // For 6-axis
    std::cout << "Cleared recorded positions for both motors." << std::endl;
    // std::cout << "Cleared recorded positions for all 6 motors." << std::endl;  // For 6-axis
}

void handleGetPos() {
    int pos1 = 0, pos2 = 0;
    if (FAS_GetActualPos(nPortID1, iSlaveNo1, &pos1) != FMM_OK ||
        FAS_GetActualPos(nPortID2, iSlaveNo2, &pos2) != FMM_OK)
    {
        std::cout << "Failed to read current positions." << std::endl;
        return;
    }

    std::cout << "Current positions - Motor1: " << pos1 << ", Motor2: " << pos2 << std::endl;
}

void handleGo() {
    // Check servo status for both motors
    EZISERVO_AXISSTATUS st1, st2;
    // EZISERVO_AXISSTATUS st3, st4, st5, st6;  // For 6-axis
    
    if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) != FMM_OK ||
        FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) != FMM_OK)
        // FAS_GetAxisStatus(nPortID3, iSlaveNo3, &(st3.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID4, iSlaveNo4, &(st4.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID5, iSlaveNo5, &(st5.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID6, iSlaveNo6, &(st6.dwValue)) != FMM_OK)    // For 6-axis
    {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return;
    }
    
    if (!st1.FFLAG_SERVOON || !st2.FFLAG_SERVOON)
        // !st3.FFLAG_SERVOON || !st4.FFLAG_SERVOON || !st5.FFLAG_SERVOON || !st6.FFLAG_SERVOON)  // For 6-axis
    {
        std::cout << "One or both servos are OFF. Turn them ON before moving." << std::endl;
        // std::cout << "One or more servos are OFF. Turn them ON before moving." << std::endl;  // For 6-axis
        return;
    }

    // Take snapshots of paths to follow
    std::vector<int> path1, path2;
    // std::vector<int> path3, path4, path5, path6;  // For 6-axis
    {
        std::lock_guard<std::mutex> lk(rec_mtx);
        path1 = recorded_positions_m1;
        path2 = recorded_positions_m2;
        // path3 = recorded_positions_m3;  // For 6-axis
        // path4 = recorded_positions_m4;  // For 6-axis
        // path5 = recorded_positions_m5;  // For 6-axis
        // path6 = recorded_positions_m6;  // For 6-axis
    }

    if (path1.empty() || path2.empty())
        // path3.empty() || path4.empty() || path5.empty() || path6.empty())  // For 6-axis
    {
        std::cout << "No recorded positions for one or both motors. Use 'record' first." << std::endl;
        // std::cout << "No recorded positions for one or more motors. Use 'record' first." << std::endl;  // For 6-axis
        return;
    }

    size_t maxSteps = std::max(path1.size(), path2.size());
    // size_t maxSteps = std::max({path1.size(), path2.size(), path3.size(), path4.size(), path5.size(), path6.size()});  // For 6-axis
    
    for (size_t i = 0; i < maxSteps; ++i)
    {
        // Get current positions
        int current_pos1 = 0, current_pos2 = 0;
        // int current_pos3 = 0, current_pos4 = 0, current_pos5 = 0, current_pos6 = 0;  // For 6-axis
        FAS_GetActualPos(nPortID1, iSlaveNo1, &current_pos1);
        FAS_GetActualPos(nPortID2, iSlaveNo2, &current_pos2);
        // FAS_GetActualPos(nPortID3, iSlaveNo3, &current_pos3);  // For 6-axis
        // FAS_GetActualPos(nPortID4, iSlaveNo4, &current_pos4);  // For 6-axis
        // FAS_GetActualPos(nPortID5, iSlaveNo5, &current_pos5);  // For 6-axis
        // FAS_GetActualPos(nPortID6, iSlaveNo6, &current_pos6);  // For 6-axis
        
        // Calculate distances and adjusted velocities for synchronization
        unsigned int velocity1 = base_velocity, velocity2 = base_velocity;
        // unsigned int velocity3 = base_velocity, velocity4 = base_velocity, velocity5 = base_velocity, velocity6 = base_velocity;  // For 6-axis
        
        if (i < path1.size() && i < path2.size()) {
            int target1 = path1[i];
            int target2 = path2[i];
            int distance1 = abs(target1 - current_pos1);
            int distance2 = abs(target2 - current_pos2);
            
            printf("Step %zu: M1(%d→%d, %d units), M2(%d→%d, %d units)\n", 
                   i+1, current_pos1, target1, distance1, 
                   current_pos2, target2, distance2);
            
            // Adjust velocities to finish at the same time
            if (distance1 > 0 && distance2 > 0) {
                if (distance1 > distance2) {
                    // Motor 1 goes farther, slow down motor 2
                    velocity2 = (unsigned int)((float)distance2 * base_velocity / distance1);
                    if (velocity2 < 100) velocity2 = 100; // minimum velocity
                } else if (distance2 > distance1) {
                    // Motor 2 goes farther, slow down motor 1
                    velocity1 = (unsigned int)((float)distance1 * base_velocity / distance2);
                    if (velocity1 < 100) velocity1 = 100; // minimum velocity
                }
                printf("Adjusted velocities: M1=%u, M2=%u\n", velocity1, velocity2);
            }
        }
        
        // For 6-axis velocity synchronization:  
        // if (i < path1.size() && i < path2.size() && i < path3.size() && 
        //     i < path4.size() && i < path5.size() && i < path6.size()) {
        //     int target1 = path1[i], target2 = path2[i], target3 = path3[i];
        //     int target4 = path4[i], target5 = path5[i], target6 = path6[i];
        //     int distance1 = abs(target1 - current_pos1);
        //     int distance2 = abs(target2 - current_pos2);
        //     int distance3 = abs(target3 - current_pos3);
        //     int distance4 = abs(target4 - current_pos4);
        //     int distance5 = abs(target5 - current_pos5);
        //     int distance6 = abs(target6 - current_pos6);
        //     
        //     printf("Step %zu: M1(%d→%d,%d), M2(%d→%d,%d), M3(%d→%d,%d), M4(%d→%d,%d), M5(%d→%d,%d), M6(%d→%d,%d)\n", 
        //            i+1, current_pos1, target1, distance1, current_pos2, target2, distance2,
        //            current_pos3, target3, distance3, current_pos4, target4, distance4,
        //            current_pos5, target5, distance5, current_pos6, target6, distance6);
        //     
        //     // Find max distance for synchronization
        //     int max_distance = std::max({distance1, distance2, distance3, distance4, distance5, distance6});
        //     if (max_distance > 0) {
        //         // Scale all velocities proportionally to max distance
        //         if (distance1 > 0) {
        //             velocity1 = (unsigned int)((float)distance1 * base_velocity / max_distance);
        //             if (velocity1 < 100) velocity1 = 100;
        //         }
        //         if (distance2 > 0) {
        //             velocity2 = (unsigned int)((float)distance2 * base_velocity / max_distance);
        //             if (velocity2 < 100) velocity2 = 100;
        //         }
        //         if (distance3 > 0) {
        //             velocity3 = (unsigned int)((float)distance3 * base_velocity / max_distance);
        //             if (velocity3 < 100) velocity3 = 100;
        //         }
        //         if (distance4 > 0) {
        //             velocity4 = (unsigned int)((float)distance4 * base_velocity / max_distance);
        //             if (velocity4 < 100) velocity4 = 100;
        //         }
        //         if (distance5 > 0) {
        //             velocity5 = (unsigned int)((float)distance5 * base_velocity / max_distance);
        //             if (velocity5 < 100) velocity5 = 100;
        //         }
        //         if (distance6 > 0) {
        //             velocity6 = (unsigned int)((float)distance6 * base_velocity / max_distance);
        //             if (velocity6 < 100) velocity6 = 100;
        //         }
        //         printf("6-Axis Synchronized velocities: M1=%u, M2=%u, M3=%u, M4=%u, M5=%u, M6=%u\n", 
        //                velocity1, velocity2, velocity3, velocity4, velocity5, velocity6);
        //     }
        // }

        // Send commands to both motors if data available
        if (i < path1.size())
        {
            int target1 = path1[i];
            if (FAS_MoveSingleAxisAbsPos(nPortID1, iSlaveNo1, target1, velocity1) != FMM_OK)
            {
                printf("Motor 1 move to %d failed.\n", target1);
                break;
            }
        }
    // Blocking && Non-blocking
        if (i < path2.size())
        {
            int target2 = path2[i];
            if (FAS_MoveSingleAxisAbsPos(nPortID2, iSlaveNo2, target2, velocity2) != FMM_OK)
            {
                printf("Motor 2 move to %d failed.\n", target2);
                break;
            }
        }
        
        // if (i < path3.size())  // For 6-axis
        // {
        //     int target3 = path3[i];
        //     if (FAS_MoveSingleAxisAbsPos(nPortID3, iSlaveNo3, target3, velocity3) != FMM_OK)
        //     {
        //         printf("Motor 3 move to %d failed.\n", target3);
        //         break;
        //     }
        // }
        
        // if (i < path4.size())  // For 6-axis
        // {
        //     int target4 = path4[i];
        //     if (FAS_MoveSingleAxisAbsPos(nPortID4, iSlaveNo4, target4, velocity4) != FMM_OK)
        //     {
        //         printf("Motor 4 move to %d failed.\n", target4);
        //         break;
        //     }
        // }
        
        // if (i < path5.size())  // For 6-axis
        // {
        //     int target5 = path5[i];
        //     if (FAS_MoveSingleAxisAbsPos(nPortID5, iSlaveNo5, target5, velocity5) != FMM_OK)
        //     {
        //         printf("Motor 5 move to %d failed.\n", target5);
        //         break;
        //     }
        // }
        
        // if (i < path6.size())  // For 6-axis
        // {
        //     int target6 = path6[i];
        //     if (FAS_MoveSingleAxisAbsPos(nPortID6, iSlaveNo6, target6, velocity6) != FMM_OK)
        //     {
        //         printf("Motor 6 move to %d failed.\n", target6);
        //         break;
        //     }
        // }

        // Wait for both motors to complete this step
        bool motor1_done = (i >= path1.size()); // skip if no more data
        bool motor2_done = (i >= path2.size()); // skip if no more data
        // bool motor3_done = (i >= path3.size()); // skip if no more data  // For 6-axis
        // bool motor4_done = (i >= path4.size()); // skip if no more data  // For 6-axis
        // bool motor5_done = (i >= path5.size()); // skip if no more data  // For 6-axis
        // bool motor6_done = (i >= path6.size()); // skip if no more data  // For 6-axis
        
        while (!motor1_done || !motor2_done)
            // !motor3_done || !motor4_done || !motor5_done || !motor6_done)  // For 6-axis
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            if (!motor1_done)
            {
                if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) == FMM_OK)
                {
                    motor1_done = !st1.FFLAG_MOTIONING;
                }
            }
            
            if (!motor2_done)
            {
                if (FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) == FMM_OK)
                {
                    motor2_done = !st2.FFLAG_MOTIONING;
                }
            }
            
            // if (!motor3_done)  // For 6-axis
            // {
            //     if (FAS_GetAxisStatus(nPortID3, iSlaveNo3, &(st3.dwValue)) == FMM_OK)
            //     {
            //         motor3_done = !st3.FFLAG_MOTIONING;
            //     }
            // }
            
            // if (!motor4_done)  // For 6-axis
            // {
            //     if (FAS_GetAxisStatus(nPortID4, iSlaveNo4, &(st4.dwValue)) == FMM_OK)
            //     {
            //         motor4_done = !st4.FFLAG_MOTIONING;
            //     }
            // }
            
            // if (!motor5_done)  // For 6-axis
            // {
            //     if (FAS_GetAxisStatus(nPortID5, iSlaveNo5, &(st5.dwValue)) == FMM_OK)
            //     {
            //         motor5_done = !st5.FFLAG_MOTIONING;
            //     }
            // }
            
            // if (!motor6_done)  // For 6-axis
            // {
            //     if (FAS_GetAxisStatus(nPortID6, iSlaveNo6, &(st6.dwValue)) == FMM_OK)
            //     {
            //         motor6_done = !st6.FFLAG_MOTIONING;
            //     }
            // }
        }
    }

    // Get final positions
    int actPos1 = 0, actPos2 = 0;
    // int actPos3 = 0, actPos4 = 0, actPos5 = 0, actPos6 = 0;  // For 6-axis
    
    if (FAS_GetActualPos(nPortID1, iSlaveNo1, &actPos1) == FMM_OK &&
        FAS_GetActualPos(nPortID2, iSlaveNo2, &actPos2) == FMM_OK)
        // FAS_GetActualPos(nPortID3, iSlaveNo3, &actPos3) == FMM_OK &&  // For 6-axis
        // FAS_GetActualPos(nPortID4, iSlaveNo4, &actPos4) == FMM_OK &&  // For 6-axis
        // FAS_GetActualPos(nPortID5, iSlaveNo5, &actPos5) == FMM_OK &&  // For 6-axis
        // FAS_GetActualPos(nPortID6, iSlaveNo6, &actPos6) == FMM_OK)    // For 6-axis
    {
        std::cout << "Go complete. Final positions - Motor1: " << actPos1 
                  << ", Motor2: " << actPos2 << std::endl;
        // std::cout << "Go complete. Final positions - M1:" << actPos1 << ", M2:" << actPos2  // For 6-axis
        //           << ", M3:" << actPos3 << ", M4:" << actPos4 << ", M5:" << actPos5 
        //           << ", M6:" << actPos6 << std::endl;
    }
    else
    {
        std::cout << "Go complete." << std::endl;
    }
}

void handleMovePos() {
    // Check servo status for both motors
    EZISERVO_AXISSTATUS st1, st2;
    // EZISERVO_AXISSTATUS st3, st4, st5, st6;  // For 6-axis
    
    if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) != FMM_OK ||
        FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) != FMM_OK)
        // FAS_GetAxisStatus(nPortID3, iSlaveNo3, &(st3.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID4, iSlaveNo4, &(st4.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID5, iSlaveNo5, &(st5.dwValue)) != FMM_OK ||  // For 6-axis
        // FAS_GetAxisStatus(nPortID6, iSlaveNo6, &(st6.dwValue)) != FMM_OK)    // For 6-axis
    {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return;
    }
    
    if (!st1.FFLAG_SERVOON || !st2.FFLAG_SERVOON)
        // !st3.FFLAG_SERVOON || !st4.FFLAG_SERVOON || !st5.FFLAG_SERVOON || !st6.FFLAG_SERVOON)  // For 6-axis
    {
        std::cout << "One or both servos are OFF. Turn them ON before moving." << std::endl;
        // std::cout << "One or more servos are OFF. Turn them ON before moving." << std::endl;  // For 6-axis
        return;
    }

    // Get target positions from user
    int target_pos1, target_pos2;
    // int target_pos3, target_pos4, target_pos5, target_pos6;  // For 6-axis
    
    std::cout << "Enter target position for Motor 1: ";
    if (!(std::cin >> target_pos1)) {
        std::cout << "Invalid input for Motor 1 position." << std::endl;
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        return;
    }
    
    std::cout << "Enter target position for Motor 2: ";
    if (!(std::cin >> target_pos2)) {
        std::cout << "Invalid input for Motor 2 position." << std::endl;
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        return;
    }
    
    // std::cout << "Enter target position for Motor 3: ";  // For 6-axis
    // if (!(std::cin >> target_pos3)) {
    //     std::cout << "Invalid input for Motor 3 position." << std::endl;
    //     std::cin.clear();
    //     std::cin.ignore(10000, '\n');
    //     return;
    // }
    
    // std::cout << "Enter target position for Motor 4: ";  // For 6-axis
    // if (!(std::cin >> target_pos4)) {
    //     std::cout << "Invalid input for Motor 4 position." << std::endl;
    //     std::cin.clear();
    //     std::cin.ignore(10000, '\n');
    //     return;
    // }
    
    // std::cout << "Enter target position for Motor 5: ";  // For 6-axis
    // if (!(std::cin >> target_pos5)) {
    //     std::cout << "Invalid input for Motor 5 position." << std::endl;
    //     std::cin.clear();
    //     std::cin.ignore(10000, '\n');
    //     return;
    // }
    
    // std::cout << "Enter target position for Motor 6: ";  // For 6-axis
    // if (!(std::cin >> target_pos6)) {
    //     std::cout << "Invalid input for Motor 6 position." << std::endl;
    //     std::cin.clear();
    //     std::cin.ignore(10000, '\n');
    //     return;
    // }
    
    // Clear input buffer
    std::cin.ignore(10000, '\n');

    // Get current positions
    int current_pos1 = 0, current_pos2 = 0;
    // int current_pos3 = 0, current_pos4 = 0, current_pos5 = 0, current_pos6 = 0;  // For 6-axis
    
    if (FAS_GetActualPos(nPortID1, iSlaveNo1, &current_pos1) != FMM_OK ||
        FAS_GetActualPos(nPortID2, iSlaveNo2, &current_pos2) != FMM_OK)
        // FAS_GetActualPos(nPortID3, iSlaveNo3, &current_pos3) != FMM_OK ||  // For 6-axis
        // FAS_GetActualPos(nPortID4, iSlaveNo4, &current_pos4) != FMM_OK ||  // For 6-axis
        // FAS_GetActualPos(nPortID5, iSlaveNo5, &current_pos5) != FMM_OK ||  // For 6-axis
        // FAS_GetActualPos(nPortID6, iSlaveNo6, &current_pos6) != FMM_OK)    // For 6-axis
    {
        printf("Failed to get current positions.\n");
        return;
    }

    // Calculate distances
    int distance1 = abs(target_pos1 - current_pos1);
    int distance2 = abs(target_pos2 - current_pos2);
    // int distance3 = abs(target_pos3 - current_pos3);  // For 6-axis
    // int distance4 = abs(target_pos4 - current_pos4);  // For 6-axis
    // int distance5 = abs(target_pos5 - current_pos5);  // For 6-axis
    // int distance6 = abs(target_pos6 - current_pos6);  // For 6-axis
    
    printf("Moving: M1(%d→%d, %d units), M2(%d→%d, %d units)\n", 
           current_pos1, target_pos1, distance1, 
           current_pos2, target_pos2, distance2);
    // printf("Moving: M1(%d→%d,%d), M2(%d→%d,%d), M3(%d→%d,%d), M4(%d→%d,%d), M5(%d→%d,%d), M6(%d→%d,%d)\n",  // For 6-axis
    //        current_pos1, target_pos1, distance1, current_pos2, target_pos2, distance2,
    //        current_pos3, target_pos3, distance3, current_pos4, target_pos4, distance4,
    //        current_pos5, target_pos5, distance5, current_pos6, target_pos6, distance6);

    // Calculate synchronized velocities
    unsigned int velocity1 = base_velocity, velocity2 = base_velocity;
    // unsigned int velocity3 = base_velocity, velocity4 = base_velocity, velocity5 = base_velocity, velocity6 = base_velocity;  // For 6-axis
    
    if (distance1 > 0 && distance2 > 0) {
        if (distance1 > distance2) {
            // Motor 1 goes farther, slow down motor 2
            velocity2 = (unsigned int)((float)distance2 * base_velocity / distance1);
            if (velocity2 < 100) velocity2 = 100; // minimum velocity
        } else if (distance2 > distance1) {
            // Motor 2 goes farther, slow down motor 1
            velocity1 = (unsigned int)((float)distance1 * base_velocity / distance2);
            if (velocity1 < 100) velocity1 = 100; // minimum velocity
        } else {
            // Equal distances - both motors use base velocity
            printf("Equal distances: both motors use base velocity %u\n", base_velocity);
        }
        printf("Synchronized velocities: M1=%u, M2=%u\n", velocity1, velocity2);
    }
    
    // 6-axis velocity synchronization algorithm:
    // int max_distance = std::max({distance1, distance2, distance3, distance4, distance5, distance6});  // For 6-axis
    // if (max_distance > 0) {
    //     // Scale all velocities proportionally to max distance
    //     if (distance1 > 0) {
    //         velocity1 = (unsigned int)((float)distance1 * base_velocity / max_distance);
    //         if (velocity1 < 100) velocity1 = 100;
    //     }
    //     if (distance2 > 0) {
    //         velocity2 = (unsigned int)((float)distance2 * base_velocity / max_distance);
    //         if (velocity2 < 100) velocity2 = 100;
    //     }
    //     if (distance3 > 0) {
    //         velocity3 = (unsigned int)((float)distance3 * base_velocity / max_distance);
    //         if (velocity3 < 100) velocity3 = 100;
    //     }
    //     if (distance4 > 0) {
    //         velocity4 = (unsigned int)((float)distance4 * base_velocity / max_distance);
    //         if (velocity4 < 100) velocity4 = 100;
    //     }
    //     if (distance5 > 0) {
    //         velocity5 = (unsigned int)((float)distance5 * base_velocity / max_distance);
    //         if (velocity5 < 100) velocity5 = 100;
    //     }
    //     if (distance6 > 0) {
    //         velocity6 = (unsigned int)((float)distance6 * base_velocity / max_distance);
    //         if (velocity6 < 100) velocity6 = 100;
    //     }
    //     printf("6-Axis Synchronized velocities: M1=%u, M2=%u, M3=%u, M4=%u, M5=%u, M6=%u\n", 
    //            velocity1, velocity2, velocity3, velocity4, velocity5, velocity6);
    // }

    // Send movement commands simultaneously
    bool success = true;
    if (distance1 > 0) {
        if (FAS_MoveSingleAxisAbsPos(nPortID1, iSlaveNo1, target_pos1, velocity1) != FMM_OK) {
            printf("Motor 1 move command failed.\n");
            success = false;
        }
    }
    
    if (distance2 > 0) {
        if (FAS_MoveSingleAxisAbsPos(nPortID2, iSlaveNo2, target_pos2, velocity2) != FMM_OK) {
            printf("Motor 2 move command failed.\n");
            success = false;
        }
    }
    
    if (!success) {
        return;
    }

    // Wait for both motors to complete movement
    bool motor1_done = (distance1 == 0), motor2_done = (distance2 == 0);
    printf("Waiting for motors to complete movement...\n");
    
    while (!motor1_done || !motor2_done) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        if (!motor1_done) {
            if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) == FMM_OK) {
                motor1_done = !st1.FFLAG_MOTIONING;
                if (motor1_done) printf("Motor 1 reached target position.\n");
            }
        }
        
        if (!motor2_done) {
            if (FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) == FMM_OK) {
                motor2_done = !st2.FFLAG_MOTIONING;
                if (motor2_done) printf("Motor 2 reached target position.\n");
            }
        }
    }
    
    // Get final positions
    int final_pos1 = 0, final_pos2 = 0;
    if (FAS_GetActualPos(nPortID1, iSlaveNo1, &final_pos1) == FMM_OK &&
        FAS_GetActualPos(nPortID2, iSlaveNo2, &final_pos2) == FMM_OK) {
        printf("Movement complete. Final positions: M1=%d, M2=%d\n", final_pos1, final_pos2);
    } else {
        printf("Movement complete.\n");
    }
}

void handleGoPos() {
    // Check servo status for both motors
    EZISERVO_AXISSTATUS st1, st2;
    if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) != FMM_OK ||
        FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) != FMM_OK)
    {
        printf("Function(FAS_GetAxisStatus) failed.\n");
        return;
    }
    
    if (!st1.FFLAG_SERVOON || !st2.FFLAG_SERVOON)
    {
        std::cout << "One or both servos are OFF. Turn them ON before moving." << std::endl;
        return;
    }

    // Load position table entries for both motors
    std::vector<ITEM_NODE> tableItems1, tableItems2;
    unsigned short maxItems = 64;
    
    // Read Motor 1 table
    for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo)
    {
        ITEM_NODE nodeItem;
        if (FAS_PosTableReadItem(nPortID1, iSlaveNo1, wItemNo, &nodeItem) != FMM_OK)
        {
            break;
        }
        bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
        if (!likelyEmpty)
            tableItems1.push_back(nodeItem);
    }
    
    // Read Motor 2 table
    for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo)
    {
        ITEM_NODE nodeItem;
        if (FAS_PosTableReadItem(nPortID2, iSlaveNo2, wItemNo, &nodeItem) != FMM_OK)
        {
            break;
        }
        bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
        if (!likelyEmpty)
            tableItems2.push_back(nodeItem);
    }

    if (tableItems1.empty() && tableItems2.empty())
    {
        std::cout << "Position tables have no items to run for both motors." << std::endl;
        return;
    }

    size_t maxSteps = std::max(tableItems1.size(), tableItems2.size());
    
    for (size_t idx = 0; idx < maxSteps; ++idx)
    {
        // Send commands to both motors if data available
        if (idx < tableItems1.size())
        {
            const ITEM_NODE& node1 = tableItems1[idx];
            unsigned int velocity1 = (node1.dwMoveSpd > 0) ? node1.dwMoveSpd : 1000;
            int target1 = node1.lPosition;

            if (FAS_MoveSingleAxisAbsPos(nPortID1, iSlaveNo1, target1, velocity1) != FMM_OK)
            {
                printf("Motor 1 move to %d failed.\n", target1);
                break;
            }
        }
        
        if (idx < tableItems2.size())
        {
            const ITEM_NODE& node2 = tableItems2[idx];
            unsigned int velocity2 = (node2.dwMoveSpd > 0) ? node2.dwMoveSpd : 1000;
            int target2 = node2.lPosition;

            if (FAS_MoveSingleAxisAbsPos(nPortID2, iSlaveNo2, target2, velocity2) != FMM_OK)
            {
                printf("Motor 2 move to %d failed.\n", target2);
                break;
            }
        }

        // Wait for both motors to complete this step
        bool motor1_done = (idx >= tableItems1.size());
        bool motor2_done = (idx >= tableItems2.size());
        
        while (!motor1_done || !motor2_done)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            if (!motor1_done)
            {
                if (FAS_GetAxisStatus(nPortID1, iSlaveNo1, &(st1.dwValue)) == FMM_OK)
                {
                    motor1_done = !st1.FFLAG_MOTIONING;
                }
            }
            
            if (!motor2_done)
            {
                if (FAS_GetAxisStatus(nPortID2, iSlaveNo2, &(st2.dwValue)) == FMM_OK)
                {
                    motor2_done = !st2.FFLAG_MOTIONING;
                }
            }
        }
    }

    // Get final positions
    int actPos1 = 0, actPos2 = 0;
    if (FAS_GetActualPos(nPortID1, iSlaveNo1, &actPos1) == FMM_OK &&
        FAS_GetActualPos(nPortID2, iSlaveNo2, &actPos2) == FMM_OK)
    {
        std::cout << "Go pos complete. Final positions - Motor1: " << actPos1 
                  << ", Motor2: " << actPos2 << std::endl;
    }
    else
    {
        std::cout << "Go pos complete." << std::endl;
    }
}

void handlePosTable() {
    // Take snapshots of recorded positions
    std::vector<int> path1, path2;
    {
        std::lock_guard<std::mutex> lk(rec_mtx);
        path1 = recorded_positions_m1;
        path2 = recorded_positions_m2;
    }

    if (path1.empty() && path2.empty())
    {
        std::cout << "No recorded positions to write for either motor. Use 'record' first." << std::endl;
        return;
    }

    unsigned short startItem = 1;
    unsigned int wrote1 = 0, wrote2 = 0;
    
    // Write Motor 1 table
    for (size_t i = 0; i < path1.size(); ++i)
    {
        unsigned short wItemNo = static_cast<unsigned short>(startItem + i);
        ITEM_NODE nodeItem;
        if (FAS_PosTableReadItem(nPortID1, iSlaveNo1, wItemNo, &nodeItem) != FMM_OK)
        {
            printf("Motor 1 FAS_PosTableReadItem failed at item %u.\n", wItemNo);
            break;
        }

        nodeItem.dwMoveSpd = 1000;
        nodeItem.lPosition = path1[i];
        nodeItem.wBranch = 0;
        nodeItem.wContinuous = 0;

        if (FAS_PosTableWriteItem(nPortID1, iSlaveNo1, wItemNo, &nodeItem) != FMM_OK)
        {
            printf("Motor 1 FAS_PosTableWriteItem failed at item %u.\n", wItemNo);
            break;
        }
        ++wrote1;
    }
    
    // Write Motor 2 table
    for (size_t i = 0; i < path2.size(); ++i)
    {
        unsigned short wItemNo = static_cast<unsigned short>(startItem + i);
        ITEM_NODE nodeItem;
        if (FAS_PosTableReadItem(nPortID2, iSlaveNo2, wItemNo, &nodeItem) != FMM_OK)
        {
            printf("Motor 2 FAS_PosTableReadItem failed at item %u.\n", wItemNo);
            break;
        }

        nodeItem.dwMoveSpd = 1000;
        nodeItem.lPosition = path2[i];
        nodeItem.wBranch = 0;
        nodeItem.wContinuous = 0;

        if (FAS_PosTableWriteItem(nPortID2, iSlaveNo2, wItemNo, &nodeItem) != FMM_OK)
        {
            printf("Motor 2 FAS_PosTableWriteItem failed at item %u.\n", wItemNo);
            break;
        }
        ++wrote2;
    }
    
    std::cout << "Position tables updated. Motor 1 items: " << wrote1 
              << ", Motor 2 items: " << wrote2 << std::endl;
}

void handlePrintTable() {
    unsigned short maxItems = 64;
    
    std::cout << "\n=== Motor 1 Position Table ===" << std::endl;
    unsigned int shown1 = 0;
    for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo)
    {
        ITEM_NODE nodeItem;
        if (FAS_PosTableReadItem(nPortID1, iSlaveNo1, wItemNo, &nodeItem) != FMM_OK)
        {
            printf("Motor 1 FAS_PosTableReadItem failed at item %u.\n", wItemNo);
            break;
        }

        bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
        if (likelyEmpty)
            continue;

        printf("M1 Item %u: Pos=%d, MoveSpd=%u, Cmd=%u, Wait=%u, Cont=%u, Branch=%u\n",
               wItemNo, nodeItem.lPosition, nodeItem.dwMoveSpd,
               nodeItem.wCommand, nodeItem.wWaitTime,
               nodeItem.wContinuous, nodeItem.wBranch);
        ++shown1;
    }
    if (shown1 == 0)
        std::cout << "No non-empty items found for Motor 1." << std::endl;
        
    std::cout << "\n=== Motor 2 Position Table ===" << std::endl;
    unsigned int shown2 = 0;
    for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo)
    {
        ITEM_NODE nodeItem;
        if (FAS_PosTableReadItem(nPortID2, iSlaveNo2, wItemNo, &nodeItem) != FMM_OK)
        {
            printf("Motor 2 FAS_PosTableReadItem failed at item %u.\n", wItemNo);
            break;
        }

        bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
        if (likelyEmpty)
            continue;

        printf("M2 Item %u: Pos=%d, MoveSpd=%u, Cmd=%u, Wait=%u, Cont=%u, Branch=%u\n",
               wItemNo, nodeItem.lPosition, nodeItem.dwMoveSpd,
               nodeItem.wCommand, nodeItem.wWaitTime,
               nodeItem.wContinuous, nodeItem.wBranch);
        ++shown2;
    }
    if (shown2 == 0)
        std::cout << "No non-empty items found for Motor 2." << std::endl;
}

void cleanup() {
    if (recording.load()) {
        recording.store(false);
        if (rec_thread.joinable()) rec_thread.join();
    }
    FAS_Close(nPortID1);
    FAS_Close(nPortID2);
    // FAS_Close(nPortID3);  // For 6-axis
    // FAS_Close(nPortID4);  // For 6-axis
    // FAS_Close(nPortID5);  // For 6-axis
    // FAS_Close(nPortID6);  // For 6-axis
}
