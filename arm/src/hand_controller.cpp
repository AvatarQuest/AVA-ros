#include <hand_controller/HandController.hpp>
#include "DynamixelMotor.hpp"
#include "DynamixelHelper.hpp"
#include <ros/ros.h>

struct AX12_TABLE : AddressTableBase() {
    TORQ
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hand_controller");
    ros::NodeHandle nh;



}