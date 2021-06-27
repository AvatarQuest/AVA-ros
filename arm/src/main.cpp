#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "dynamixel_sdk.h"  
#include "ArmController.hpp"

int main() {
    const double a1 = 13;
    const double a2 = 12.3;
    const double a3 = 8.5;
    double offsets[] = {0, -14.25, 14.25, 0, 0};

    const std::string port = "/dev/ttyUSB0";
    const uint baudrate = 57600;
    const AddressTableBase table = XM430W350T_TABLE();
    std::unordered_map<MotorIdentifier, AddressTableBase, MotorIndentifierHasher> motors;

    MotorIdentifier rotation_motor = MotorIdentifier(11, baudrate, port);
    MotorIdentifier shoulder =  MotorIdentifier(12, baudrate, port);
    MotorIdentifier elbow = MotorIdentifier(13, baudrate, port);
    MotorIdentifier wrist = MotorIdentifier(14, baudrate, port);
    MotorIdentifier claw = MotorIdentifier(15, baudrate, port);
    
    ArmController controller = ArmController(rotation_motor, shoulder, elbow, wrist, claw, table, a1, a2, a3);
    controller.setOffsets(offsets);
    controller.setDebug(true);

    controller.setAllTorque(true);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    controller.moveArm(7, 7);
    controller.setRotationMotorAngle(230);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    controller.moveArm(0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));   
    controller.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    controller.moveArm(5, 5);
    controller.setRotationMotorAngle(160); 
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    controller.moveArm(8, 3);
    controller.setRotationMotorAngle(230); 
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // controller.moveArm(10, 5);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    // controller.moveArm(10, 10);
    // helper.printAll();
}
