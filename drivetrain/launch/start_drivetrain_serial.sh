#!/bin/bash
rosrun rosserial_python serial_node.py /dev/ttyACM1 __name:=drivetrain_serial
