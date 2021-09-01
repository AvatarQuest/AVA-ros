#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <unordered_map>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>
#include <chrono>
#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"                         // Uses Dynamixel SDK library
#include "DynamixelHelper.hpp"

#define PI 3.14159265

class ArmController {
    private: 
        DynamixelHelper helper;
        MotorIdentifier rotation_motor, shoulder, elbow, wrist, claw;
        double a1, a2, a3;
        double offsets[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        bool debug = false;

        /**
         * @brief Converts between the dynamixel unit of 0-4096 to angle in degrees
         * 
         * @param theta The angle to convert
         * @return double The dynamixel unit output
         */
        double thetaToPos(double theta) {
            return (theta / 360) * 4096;
        }

    public:
        /**
         * @brief Construct a new Arm Controller object
         * 
         * @param helper 
         
         */

        /**
         * @brief Construct a new Arm Controller object
         * 
         * @param rotation_motor The bottom rotation motor that can move the entire arm
         * @param shoulder The second motor from the bottom
         * @param elbow The motor in the middle of the arm that acts like an elbow
         * @param wrist The motor that controls the movement of the claw
         * @param claw The claw motor
         * @param table The address for all the motors
         * @param a1 The length for a1
         * @param a2 The length for a2
         * @param a3 The lenght for a3
         */
        ArmController(MotorIdentifier rotation_motor, MotorIdentifier shoulder, MotorIdentifier elbow, MotorIdentifier wrist, MotorIdentifier claw, AddressTableBase table, double a1, double a2, double a3) {
            this->rotation_motor = rotation_motor;
            this->shoulder = shoulder;
            this->elbow = elbow;
            this->wrist = wrist;
            this->claw = claw;

            this->a1 = a1;
            this->a2 = a2;
            this->a3 = a3;

            std::unordered_map<MotorIdentifier, AddressTableBase, MotorIndentifierHasher> motor_map;

            motor_map[rotation_motor] = table;
            motor_map[shoulder] = table;
            motor_map[elbow] = table;
            motor_map[wrist] = table;
            motor_map[claw] = table;

            helper = DynamixelHelper(motor_map);
        }

       /**
         * @brief Construct a new Arm Controller object, this construtor can be used when you are using different motor addresstables and can supply a custom helper object
         * 
         * @param helper A DynamixelHelper object with the 5 motors for the arm
         * @param rotation_motor The bottom rotation motor that can move the entire arm
         * @param shoulder The second motor from the bottom
         * @param elbow The motor in the middle of the arm that acts like an elbow
         * @param wrist The motor that controls the movement of the claw
         * @param claw The claw motor
         * @param a1 The length for a1
         * @param a2 The length for a2
         * @param a3 The lenght for a3
         */ 
        ArmController(DynamixelHelper helper, MotorIdentifier rotation_motor, MotorIdentifier shoulder, MotorIdentifier elbow, MotorIdentifier wrist, MotorIdentifier claw, double a1, double a2, double a3) {
            this->helper = helper;

            this->rotation_motor = rotation_motor;
            this->shoulder = shoulder;
            this->elbow = elbow;
            this->wrist = wrist;
            this->claw = claw;

            this->a1 = a1;
            this->a2 = a2;
            this->a3 = a3;
        }

        void setOffsets(double new_offsets[]) {
            offsets[0] = new_offsets[0];
            offsets[1] = new_offsets[1];
            offsets[2] = new_offsets[2];
            offsets[3] = new_offsets[3];
            offsets[4] = new_offsets[4];
        }

        void setDebug(bool value) {
            debug = value;
        }

        /**
         * @brief Set the angle of a motor
         * 
         * @param id The MotorIdentifier for the motor
         * @param angle the angle you need to set the motor to
         */
        void setAngle(MotorIdentifier id, double angle) {
            DynamixelMotor motor = helper.getByMotorIdentifier(id);
            if (debug) {
                std::cout << "Moving: " << motor << " to angle " << angle << std::endl;
            }
            motor.setGoal(thetaToPos(angle));
        }

        /**
         * @brief  A method to conveniently set the angle of the rotation motor with a optional offset from the setOffset method
         * 
         * @param angle The angle to set the motor to
         */
        void setRotationMotorAngle(double angle) {
           setAngle(rotation_motor, angle + offsets[0]);
        }

        /**
         * @brief A method to conveniently set the angle of the shoulder with a optional offset from the setOffset method
         * 
         * @param angle The angle to set the motor to
         */
        void setShoulderAngle(double angle) {
           setAngle(shoulder, angle + offsets[1]);
        }

        /**
         * @brief  A method to conveniently set the angle of the elbow with a optional offset from the setOffset method
         * 
         * @param angle The angle to set the motor to
         */
        void setElbowAngle(double angle) {
           setAngle(elbow, angle + offsets[2]);
        }

        /**
         * @brief A method to conveniently set the angle of the wrist with a optional offset from the setOffset method
         * 
         * @param angle The angle to set the motor to
         */
        void setWristAngle(double angle) {
           setAngle(wrist, angle + offsets[3]);
        }

        /**
         * @brief  A method to conveniently set the angle of the claw with a optional offset from the setOffset method 
         * 
         * @param angle The angle to set the motor to
         */
        void setClawAngle(double angle) {
           setAngle(claw, angle + offsets[4]);
        }

        void setAllTorque(bool value) {
            helper.getByMotorIdentifier(rotation_motor).setTorque(value);
            helper.getByMotorIdentifier(shoulder).setTorque(value);
            helper.getByMotorIdentifier(elbow).setTorque(value);
            helper.getByMotorIdentifier(wrist).setTorque(value);
            helper.getByMotorIdentifier(claw).setTorque(value);
        }
        
        /**
         * @brief Returns the angles the arm should move each motor to mvove to a specific x, y position in cm in space
         * 
         * @param px 
         * @param py 
         * @return double[] of angle values for the shoulder, elbow and wrist 
         */
        std::vector<double> ik(double px, double py) {
            double phi =((270 * PI) / 180.0);

            double wx = px - a3 * cos(phi);
            double wy = py - a3 * sin(phi);

            double delta  = pow(wx, 2) + pow(wy, 2);
            double c2 = ((delta - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2));
            double s2 = sqrt(1 - pow(c2, 2));
            double theta_2 = atan2(s2, c2);

            double s1 = ((a1 + a2 * c2) * wy - a2 * s2 * wx) / delta;
            double c1 = ((a1 + a2 * c2) * wx + a2 * s2 * wy) / delta;
            double theta_1 = atan2(s1, c1);
            double theta_3 = phi - theta_1 - theta_2;

            theta_1 = ((theta_1 * 180) / PI) + 180;
            theta_2 = ((theta_2 * 180) / PI) + 90;
            theta_3 = (theta_3 * 180) / PI;
    
            std::vector<double> angles = {theta_1, theta_2, theta_3};

            if (debug) {
                std::cout << "Theta 1: " << theta_1 << std::endl;
                std::cout << "Theta 2: " << theta_2 << std::endl;
                std::cout << "Theta 3: " << theta_3 << std::endl;
            }
            return angles;
        }
        /**
         * @brief Returns the angles the arm should move each motor to move to a specific x, y, z position in cm in space
         * @param px 
         * @param py
         * @param pz
         * @return double [] of angle values for the new arm
         */
        /*
            use the iterative strategy of IK
            two dynamixels are on the same joint so will move with the same angle
        */
        std::vector<double> ik3(double px, double py, double pz){
            
        }

        /**
         * @brief Moves the arm to a speicific x, y postion in cm using inverse kinematics
         * 
         * @param x The x poisiton in cm
         * @param y The y position in cm
         */
        void moveArm(double x, double y) {
            std::vector<double> angles = ik(x, y);
            if (std::isnan(angles[0]) || std::isnan(angles[0]) || std::isnan(angles[0])) {
                std::cout << "Invalid position, not moving the arm" << std::endl;
                return;
            } 
            if (debug) {
                std::cout << angles[0] << std::endl;
                std::cout << angles[1] << std::endl;
                std::cout << angles[2] << std::endl;
            }
                
            setShoulderAngle(angles[0]);
            setElbowAngle(angles[1]);
            setWristAngle(angles[2]);
        }
        /**
         * @brief Moves the arm to a specific x, y, z position
         * @param x The x position in cm
         * @param y The y position in cm 
         * @param z The z position in cm
         */
        void moveArm(double x, double y, double z){
            std::vector<double> angles = ik3(x,y,z);

        }
        /**
         * @brief Moves the arm to a speicific x, y postion in cm using inverse kinematics witha delay in between each movement
         * 
         * @param x The x poisiton in cm
         * @param y The y position in cm
         * @param delay The delay in milliseconds bewteen each movement
         */
        void moveArm(double x, double y, int delay) {
            std::vector<double> angles = ik(x, y);
            if (std::isnan(angles[0]) || std::isnan(angles[0]) || std::isnan(angles[0])) {
                std::cout << "Invalid position, not moving the arm" << std::endl;
                return;
            } 
            if (debug) {
                std::cout << "Shoulder moving to: " << angles[0] << std::endl;
                std::cout << "Elbow moving to: " << angles[1] << std::endl;
                std::cout << "Wrist moving to: " << angles[2] << std::endl;
            }
            
            setShoulderAngle(angles[0]);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            setElbowAngle(angles[1]);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            setWristAngle(angles[2]);
        }

        /**
         * @brief A method that "zeros" out the arm by rebooting all motors, putting the arm back to 0, 0 and rotating the roation motor to 180 degrees
         * 
         */
        void reset() {
            setAllTorque(false);

            bool error = false;
            if (!helper.getByMotorIdentifier(rotation_motor).reboot()) error = true;
            if (!helper.getByMotorIdentifier(shoulder).reboot()) error = true;
            if (!helper.getByMotorIdentifier(elbow).reboot()) error = true;
            if (!helper.getByMotorIdentifier(wrist).reboot()) error = true;
            if (!helper.getByMotorIdentifier(claw).reboot()) error = true;

            if (error) {
                std::cout << "An error occured while rebooting, not moving the arm to (0, 0)" << std::endl;
                return;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            setAllTorque(true);
            moveArm(0, 0, 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            setRotationMotorAngle(180);
        }

        /**
         * @brief Get the Helper object
         * 
         * @return DynamixelHelper 
         */
        DynamixelHelper getHelper() {
            return helper;
        }

        /**
         * @brief Get a motor from the dynamixel helper
         * 
         * @param id The motor idenfier of the motor
         * @return DynamixelMotor 
         */
        DynamixelMotor getMotor(MotorIdentifier id) {
            return helper.getByMotorIdentifier(id);
        }


};
