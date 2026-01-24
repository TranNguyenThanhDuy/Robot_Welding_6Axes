#include "axis_controller.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <limits>
#include <string>

void showCommands() {
    std::cout << "=== " << AXIS_COUNT << "-AXIS SERVO CONTROL ===" << std::endl;
    std::cout << "on: Servo on (" << AXIS_COUNT << " motors)" << std::endl;
    std::cout << "off: Servo off (" << AXIS_COUNT << " motors)" << std::endl;
    std::cout << "home: Homing all motors (Only works in servo on)" << std::endl;
    std::cout << "record: Start recording all motors (Only works in servo off)" << std::endl;
    std::cout << "stop: Stop recording" << std::endl;
    std::cout << "clear: Clear buffer" << std::endl;
    std::cout << "getpos: Print current positions of all motors" << std::endl;
    std::cout << "go: Go with positions in buffer (Only works in servo on)" << std::endl;
    std::cout << "movepos: Move motors to specified positions (Only works in servo on)"
              << std::endl;
    std::cout << "go pos: Run items saved in position table (Only works in servo on)" << std::endl;
    std::cout << "postable: Save record to position table" << std::endl;
    std::cout << "print table: Print position table" << std::endl;
    std::cout << "setpos: Set all axes positions to 0" << std::endl;
    std::cout << "q: Quit" << std::endl;
}

int main() {
    AxisController controller;
    controller.initializeSystem();
    showCommands();

    std::string line;
    while (true) {
        if (!std::getline(std::cin, line)) break;
        std::string cmd = line;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        if (cmd == "q") break;
        if (cmd.empty()) continue;

        if (cmd == "on") {
            controller.servoOn();
        } else if (cmd == "off") {
            controller.servoOff();
        } else if (cmd == "home") {
            controller.home();
        } else if (cmd == "record") {
            controller.record();
        } else if (cmd == "stop") {
            controller.stop();
        } else if (cmd == "clear") {
            controller.clear();
        } else if (cmd == "getpos") {
            AxisPositions pos{};
            if (!controller.getPos(pos)) {
                std::cout << "Failed to read current positions." << std::endl;
                continue;
            }

            std::cout << "Current positions - ";
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                std::cout << controller.axisName(i) << ": " << pos[i];
                if (i + 1 < AXIS_COUNT) std::cout << ", ";
            }
            std::cout << std::endl;
        } else if (cmd == "go") {
            controller.go();
        } else if (cmd == "movepos" || cmd == "move pos") {
            AxisPositions targets{};
            bool inputOk = true;
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                std::cout << "Enter target position for " << controller.axisName(i) << ": ";
                if (!(std::cin >> targets[i])) {
                    std::cout << "Invalid input for " << controller.axisName(i) << " position."
                              << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    inputOk = false;
                    break;
                }
            }
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (inputOk) {
                controller.movePos(targets);
            }
        } else if (cmd == "go pos" || cmd == "gopos") {
            controller.goPos();
        } else if (cmd == "postable") {
            controller.posTable();
        } else if (cmd == "print table" || cmd == "printtable") {
            controller.printTable();
        } else if (cmd == "setpos" || cmd == "set pos") {
            controller.setOriginPos();
        } else {
            std::cout << "Unknown command. Use 'on', 'off', 'home', 'record', 'stop', 'clear', "
                         "'getpos', 'go', 'movepos', 'go pos', 'postable', 'print table', "
                         "'setpos', or 'q'."
                      << std::endl;
        }
    }

    return 0;
}
