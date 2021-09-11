#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>

#include <ros/ros.h>

#define PI 3.14159265

namespace ik3d {
    class IKServer {
        private: 
            // ros::NodeHandle nh;
            double a1, a2, a3;
            double offsets[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
            bool debug = false;

        public:
            IKServer() {}
            /**
            * @brief Construct a new Arm Controller object
            * 
            * @param a1 The length for the first part of the elbow
            * @param a2 The length for the forearm
            * @param a3 The lenght for the wrist to the end of the endeffector
            */
            IKServer(double a1, double a2, double a3) {
                // this->nh = nh;

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

            /**
            * @brief  A method to see optional debug messages while the code is runnings
            * 
            * @param angle The angle to set the motor to
            */ 
            void setDebug(bool value) {
                debug = value;
            }

            /**
            * @brief  A method to conveniently set the angle of the rotation motor with a optional offset from the setOffset method
            * 
            * @param angle The angle to set the motor to
            */
            virtual void setZAngle(double angle);

            /**
            * @brief A method to conveniently set the angle of the shoulder with a optional offset from the setOffset method
            * 
            * @param angle The angle to set the motor to
            */
            virtual void setShoulderAngle(double angle);

            /**
            * @brief  A method to conveniently set the angle of the elbow with a optional offset from the setOffset method
            * 
            * @param angle The angle to set the motor to
            */
            virtual void setElbowAngle(double angle);

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
                //90-theta1
                //(90-theta1) * (-1) - theta1 is negative
                
                theta_1 = (theta_1 * 180) / PI;
                theta_2 = -((theta_2 * 180) / PI) + 180;
                theta_3 = (theta_3 * 180) / PI;
        
                std::vector<double> angles = {theta_1, theta_2, theta_3};

                if (debug) {
                    ROS_INFO("Theta 1: %.2f", theta_1);
                    ROS_INFO("Theta 2: %.2f", theta_2);
                    ROS_INFO("Theta 3: %.2f", theta_3);
                }
                return angles;
            }

            /** 
             * @brief moves the arm in 3 dimesnions
             * 
             */
            void moveArm(double x, double y, double z) {
                double x_component = x;
                double y_component = y == 0 ? 0 : (y / abs(y)) * sqrt(pow(y, 2) + pow(z, 2));
                double z_angle = abs(x)+abs(y)+abs(z) == 0 ? 0 : asin(z/sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2))) * (180/PI); 
                if (debug) {
                    ROS_INFO("POSITION: %.2f, %.2f, %.2f", x, y, z);
                    ROS_INFO("TRANSLATED TO: %.2f, %.2f", x_component, y_component, z_angle);
                }
                if (moveArm(x_component, y_component)) {
                    setZAngle(z_angle + offsets[3]);
                }
            }

            /**
            * @brief Moves the arm to a speicific x, y postion in cm using inverse kinematics
            * 
            * @param x The x poisiton in cm
            * @param y The y position in cm
            */
            bool moveArm(double x, double y) {
                std::vector<double> angles = ik(x, y);
                if (std::isnan(angles[0]) || std::isnan(angles[1]) || std::isnan(angles[2])) {
                    ROS_WARN("Invalid position, not moving the arm");
                    return false;
                } 
                if (debug) {
                    ROS_INFO("ANGLES: %.2f, %.2f, %.2f", angles[0], angles[1], angles[2]);
                }
                    
                setShoulderAngle(angles[0]);
                setElbowAngle(angles[1]);
                return true;
            }
    };
}