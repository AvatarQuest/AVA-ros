#include <unordered_map>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>
#include <chrono>

#include <ros/ros.h>

namespace hand_controller {
    class HandController {
        private:
            ros::NodeHandle nh;
            double offsets[5];
        public: 
            HandController(ros::NodeHandle &nh) {
                this->nh = nh;
            }

            virtual void setIndex(double angle);

            virtual void setMiddle(double angle);

            virtual void setRing(double angle);

            virtual void setPinky(double angle);

            virtual void setThumb(double angle);
    }
}