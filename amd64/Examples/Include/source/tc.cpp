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

int main()
{
    const wchar_t* sPort = L"ttyUSB0";
    unsigned int dwBaudRate = 115200;
    int nPortID = 0;
    unsigned char iSlaveNo = 0;

    // Connect and show drive info via helper
    Driver_Connection(sPort, dwBaudRate, nPortID, iSlaveNo);

    std::vector<int> recorded_positions;
    std::vector<int> recorded_velocities; // captured actual velocity alongside position
    std::mutex rec_mtx;
    std::atomic<bool> recording{false};
    std::thread rec_thread;

    auto startRecording = [&]() {
        if (recording.load()) {
            std::cout << "Already recording." << std::endl;
            return;
        }
        recording.store(true);
        rec_thread = std::thread([&]() {
            int lastPos = 0;
            bool hasLast = false;
            while (recording.load()) {
                int pos = 0;
                if (FAS_GetActualPos(nPortID, iSlaveNo, &pos) == FMM_OK) {
                    if (!hasLast || pos != lastPos) {
                        int vel = 0;
                        if (FAS_GetActualVel(nPortID, iSlaveNo, &vel) != FMM_OK) {
                            vel = 0; // fallback if velocity read fails
                        }
                        std::lock_guard<std::mutex> lk(rec_mtx);
                        recorded_positions.push_back(pos);
                        recorded_velocities.push_back(vel);
                        lastPos = pos;
                        hasLast = true;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        std::cout << "Recording started (capturing pos + vel). Type 'stop' to end." << std::endl;
    };

    auto stopRecording = [&]() {
        if (!recording.load()) {
            std::cout << "Not recording." << std::endl;
            return;
        }
        recording.store(false);
        if (rec_thread.joinable()) rec_thread.join();
        std::lock_guard<std::mutex> lk(rec_mtx);
        std::cout << "Recording stopped. Samples: " << recorded_positions.size() << std::endl;
        if (!recorded_positions.empty()) {
            std::cout << "Positions: [";
            for (size_t i = 0; i < recorded_positions.size(); ++i) {
                std::cout << recorded_positions[i];
                if (i + 1 < recorded_positions.size()) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            if (recorded_velocities.size() == recorded_positions.size()) {
                std::cout << "Velocities: [";
                for (size_t i = 0; i < recorded_velocities.size(); ++i) {
                    std::cout << recorded_velocities[i];
                    if (i + 1 < recorded_velocities.size()) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            }
        }
    };

    // std::cout << "Commands: on, off, home, record, stop, clear, go, postable, print table, q" << std::endl;
    std::cout << "on: Servo on"<<std::endl;
    std::cout << "off: Servo off"<<std::endl;
    std::cout << "home: homing (Only work in servo on)"<<std::endl;
    std::cout << "record: start recoring (Only work in servo off)"<<std::endl;
    std::cout << "stop: stop recoring"<<std::endl;
    std::cout << "clear: clear buffer"<<std::endl;
    std::cout << "go: go with position in buffer (Only work in servo on)"<<std::endl;
    std::cout << "go pos: go through items saved in position table (Only work in servo on)"<<std::endl;
    std::cout << "postable: save record to position table"<<std::endl;
    std::cout << "print table: print position table"<<std::endl;
    std::string line;
    while (true)
    {
        if (!std::getline(std::cin, line)) break;
        std::string cmd = line;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c){ return std::tolower(c); });
        if (cmd == "q") break;
        if (cmd.empty()) continue;

        if (cmd == "on")
        {
            if (!ServoOn(nPortID, iSlaveNo))
                printf("ServoOn failed.\n");
        }
        else if (cmd == "off")
        {
            if (!ServoOff(nPortID, iSlaveNo))
                printf("ServoOff failed.\n");
        }
        else if (cmd == "home")
        {
            EZISERVO_AXISSTATUS st;
            if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(st.dwValue)) != FMM_OK)
            {
                printf("Function(FAS_GetAxisStatus) failed.\n");
                continue;
            }
            if (!st.FFLAG_SERVOON)
            {
                std::cout << "Servo is OFF. Turn it ON before homing." << std::endl;
                continue;
            }

            const unsigned int velocity = 1000; // default homing move speed
            if (FAS_MoveSingleAxisAbsPos(nPortID, iSlaveNo, 0, velocity) != FMM_OK)
            {
                printf("Function(FAS_MoveSingleAxisAbsPos) failed.\n");
                continue;
            }

            // Wait until motion completes
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(st.dwValue)) != FMM_OK)
                {
                    printf("Function(FAS_GetAxisStatus) failed while waiting.\n");
                    break;
                }
            } while (st.FFLAG_MOTIONING);

            int actPos = 0;
            if (FAS_GetActualPos(nPortID, iSlaveNo, &actPos) == FMM_OK)
            {
                std::cout << "Homing complete. ActualPos = " << actPos << std::endl;
            }
            else
            {
                std::cout << "Homing complete." << std::endl;
            }
        }
        else if (cmd == "record")
        {
            startRecording();
        }
        else if (cmd == "stop")
        {
            stopRecording();
        }
        else if (cmd == "clear")
        {
            std::lock_guard<std::mutex> lk(rec_mtx);
            recorded_positions.clear();
            recorded_velocities.clear();
            std::cout << "Cleared recorded positions." << std::endl;
        }
        else if (cmd == "go")
        {
            // Ensure servo is ON
            EZISERVO_AXISSTATUS st;
            if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(st.dwValue)) != FMM_OK)
            {
                printf("Function(FAS_GetAxisStatus) failed.\n");
                continue;
            }
            if (!st.FFLAG_SERVOON)
            {
                std::cout << "Servo is OFF. Turn it ON before moving." << std::endl;
                continue;
            }

            // Take a snapshot of the path to follow
            std::vector<int> path;
            {
                std::lock_guard<std::mutex> lk(rec_mtx);
                path = recorded_positions;
            }

            if (path.empty())
            {
                std::cout << "No recorded positions. Use 'record' first." << std::endl;
                continue;
            }

            const unsigned int velocity = 1000; // default move speed
            for (size_t i = 0; i < path.size(); ++i)
            {
                int target = path[i];
                if (FAS_MoveSingleAxisAbsPos(nPortID, iSlaveNo, target, velocity) != FMM_OK)
                {
                    printf("Move to %d failed.\n", target);
                    break;
                }

                // Wait until motion completes
                do {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(st.dwValue)) != FMM_OK)
                    {
                        printf("Function(FAS_GetAxisStatus) failed while waiting.\n");
                        break;
                    }
                } while (st.FFLAG_MOTIONING);
            }

            int actPos = 0;
            if (FAS_GetActualPos(nPortID, iSlaveNo, &actPos) == FMM_OK)
            {
                std::cout << "Go complete. Final ActualPos = " << actPos << std::endl;
            }
            else
            {
                std::cout << "Go complete." << std::endl;
            }
        }
        else if (cmd == "go pos" || cmd == "gopos")
        {
            // Ensure servo is ON
            EZISERVO_AXISSTATUS st;
            if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(st.dwValue)) != FMM_OK)
            {
                printf("Function(FAS_GetAxisStatus) failed.\n");
                continue;
            }
            if (!st.FFLAG_SERVOON)
            {
                std::cout << "Servo is OFF. Turn it ON before moving." << std::endl;
                continue;
            }

            // Load position table entries
            std::vector<ITEM_NODE> tableItems;
            unsigned short maxItems = 64;
            for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo)
            {
                ITEM_NODE nodeItem;
                if (FAS_PosTableReadItem(nPortID, iSlaveNo, wItemNo, &nodeItem) != FMM_OK)
                {
                    printf("FAS_PosTableReadItem failed at item %u.\n", wItemNo);
                    break;
                }

                bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
                if (likelyEmpty)
                    continue;

                tableItems.push_back(nodeItem);
            }

            if (tableItems.empty())
            {
                std::cout << "Position table has no items to run." << std::endl;
                continue;
            }

            for (size_t idx = 0; idx < tableItems.size(); ++idx)
            {
                const ITEM_NODE& node = tableItems[idx];
                unsigned int velocity = (node.dwMoveSpd > 0) ? node.dwMoveSpd : 1000;
                int target = node.lPosition;

                if (FAS_MoveSingleAxisAbsPos(nPortID, iSlaveNo, target, velocity) != FMM_OK)
                {
                    printf("Move to %d failed.\n", target);
                    break;
                }

                do {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(st.dwValue)) != FMM_OK)
                    {
                        printf("Function(FAS_GetAxisStatus) failed while waiting.\n");
                        break;
                    }
                } while (st.FFLAG_MOTIONING);
            }

            int actPos = 0;
            if (FAS_GetActualPos(nPortID, iSlaveNo, &actPos) == FMM_OK)
            {
                std::cout << "Go pos complete. Final ActualPos = " << actPos << std::endl;
            }
            else
            {
                std::cout << "Go pos complete." << std::endl;
            }
        }
        else if (cmd == "postable")
        {
            // Take a snapshot of recorded positions
            std::vector<int> path;
            {
                std::lock_guard<std::mutex> lk(rec_mtx);
                path = recorded_positions;
            }

            if (path.empty())
            {
                std::cout << "No recorded positions to write. Use 'record' first." << std::endl;
                continue;
            }

            unsigned short startItem = 1; // start from item #1
            unsigned int wrote = 0;
            for (size_t i = 0; i < path.size(); ++i)
            {
                unsigned short wItemNo = static_cast<unsigned short>(startItem + i);
                ITEM_NODE nodeItem;
                if (FAS_PosTableReadItem(nPortID, iSlaveNo, wItemNo, &nodeItem) != FMM_OK)
                {
                    printf("FAS_PosTableReadItem failed at item %u.\n", wItemNo);
                    break;
                }

                nodeItem.dwMoveSpd = 1000;               // default speed
                nodeItem.lPosition = path[i];             // recorded absolute position
                nodeItem.wBranch = 0;                     // no branch
                nodeItem.wContinuous = 0;                 // stop at each item by default

                if (FAS_PosTableWriteItem(nPortID, iSlaveNo, wItemNo, &nodeItem) != FMM_OK)
                {
                    printf("FAS_PosTableWriteItem failed at item %u.\n", wItemNo);
                    break;
                }
                ++wrote;
            }
            std::cout << "Position table updated. Items written: " << wrote << std::endl;
        }
        else if (cmd == "print table" || cmd == "printtable")
        {
            unsigned short maxItems = 64; // inspect first 64 items
            unsigned int shown = 0;
            for (unsigned short wItemNo = 1; wItemNo <= maxItems; ++wItemNo)
            {
                ITEM_NODE nodeItem;
                if (FAS_PosTableReadItem(nPortID, iSlaveNo, wItemNo, &nodeItem) != FMM_OK)
                {
                    printf("FAS_PosTableReadItem failed at item %u.\n", wItemNo);
                    break;
                }

                // Heuristic: skip likely empty items
                bool likelyEmpty = (nodeItem.lPosition == 0 && nodeItem.dwMoveSpd == 0);
                if (likelyEmpty)
                    continue;

                printf("Item %u: Pos=%d, MoveSpd=%u, Cmd=%u, Wait=%u, Cont=%u, Branch=%u\n",
                       wItemNo,
                       nodeItem.lPosition,
                       nodeItem.dwMoveSpd,
                       nodeItem.wCommand,
                       nodeItem.wWaitTime,
                       nodeItem.wContinuous,
                       nodeItem.wBranch);
                ++shown;
            }
            if (shown == 0)
                std::cout << "No non-empty items found in first 64 entries." << std::endl;
        }
        else
        {
            std::cout << "Unknown command. Use 'on', 'off', 'home', 'record', 'stop', 'clear', 'go', 'go pos', 'postable', 'print table', or 'q'." << std::endl;
        }
    }
    if (recording.load()) {
        recording.store(false);
        if (rec_thread.joinable()) rec_thread.join();
    }
    FAS_Close(nPortID);
    return 0;
}
