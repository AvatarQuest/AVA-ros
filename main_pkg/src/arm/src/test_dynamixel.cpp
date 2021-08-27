#include <iostream>
#include "DynamixelHelper.hpp"

#define ID 1
#define BAUDRATE 57600
#define PORT "/dev/ttyUSB0"

struct AX12A_TABLE: AddressTableBase {
    AX12A_TABLE() {
        TORQUE_ENABLE = 24;
        GOAL_POSITION = 30;
        PRESENT_POSITION = 36;
    }
} TABLE;

int main() {
    DynamixelMotor motor = DynamixelMotor(ID, BAUDRATE, PORT, AX12A_TABLE());
    std::cout<<"starting motor"<<"\n";
    motor.init();
    motor.setGoal(100);
}