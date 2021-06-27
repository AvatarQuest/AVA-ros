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

#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "DynamixelMotor.hpp"

/**
 * @brief A type to handle the location indentifiers of a motor: id, baudrate, and port
 * 
 */
typedef struct MotorIdentifier {
    /**
     * @brief The id of the motor
     * 
     */
    uint id;
    /**
     * @brief The baudrate of the motor
     * 
     */
    uint baudrate;
    /**
     * @brief The port of the motor
     * 
     */
    std::string port;

    MotorIdentifier() {}

    MotorIdentifier(int id, int baudrate, std::string port) {
        this->id = id;
        this->baudrate = baudrate;
        this->port = port;
    }

    /**
     * @brief Checks if two indetifiers point to the same motor
     * 
     * @param other The other motor
     * @return true The other identifier points to the same objects
     * @return false The other identifier does not point to the same objects
     */
    bool operator==(const MotorIdentifier& other) const { 
        return id == other.id && baudrate == other.baudrate && port == other.port; 
    } 
} MotorIdentifier;

typedef struct Port {
    uint baudrate;
    std::string port;

    Port() {}
    Port(uint baudrate, std::string port) {
        this->baudrate = baudrate;
        this->port = port;
    }

    /**
     * @brief Checks if two indetifiers point to the same motor
     * 
     * @param other The other motor
     * @return true The other identifier points to the same objects
     * @return false The other identifier does not point to the same objects
     */
    bool operator==(const Port& other) const { 
        return baudrate == other.baudrate && port == other.port; 
    } 
} Port;


/**
 * @brief A class to create unqiue hashes for a MotorIdentifier
 * 
 */
class MotorIndentifierHasher { 
    public: 
        /**
         * @brief The method to create a hash based on id, baudrate, and port
         * 
         * @param m The indentifier to hash
         * @return size_t The hash
         */
        size_t operator()(const MotorIdentifier& m) const
        { 
            return ((std::hash<int>()(m.id)
            ^ (std::hash<int>()(m.baudrate) << 1)) >> 1)
            ^ (std::hash<std::string>()(m.port) << 1);
        } 
};

/**
 * @brief A class to create unqiue hashes for a Port object 
 * 
 */
class PortHasher { 
    public: 
        /**
         * @brief The method to create a hash based on baudrate, and port
         * 
         * @param m The port to hash
         * @return size_t The hash
         */
        size_t operator()(const Port& m) const
        { 
            return ((std::hash<int>()(m.baudrate)
            ^ (std::hash<std::string>()(m.port) << 1)));
        } 
};

class DynamixelHelper {
    private:
        /**
         * @brief A hash map to deal with each motor with they key as a MotorIdentifier and the value as the DynamixelMotor object
         * 
         */
        std::unordered_map<MotorIdentifier, DynamixelMotor, MotorIndentifierHasher> motors;

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
        DynamixelHelper() {}

        /**
         * @brief Construct a new Dynamixel Helper object
         * 
         * @param dynamixel_motors A vector of dynamixel motors that need to be controlled
         */
        DynamixelHelper(std::vector<DynamixelMotor> dynamixel_motors) {
            for (DynamixelMotor &motor : dynamixel_motors) {
                MotorIdentifier identifier;
                identifier.id = motor.getId();
                identifier.baudrate = motor.getBaudrate();
                identifier.port = motor.getPort();

                motors.insert(std::make_pair(identifier, motor));
            }
        }

       /**
        * @brief Construct a new Dynamixel Helper object and shares a pool of port handlers betweeen motors, use this over the other contructor ehen you can
        * 
        * @param motor_map A map including a motor identifier as the key and the address table of the corrospoding motor as the value
        */
        DynamixelHelper(std::unordered_map<MotorIdentifier, AddressTableBase, MotorIndentifierHasher> motor_map) {
            std::unordered_map<Port, dynamixel::PortHandler*, PortHasher> portHandlerPool = {};

            for (auto motor : motor_map) {
                MotorIdentifier id = motor.first;
                dynamixel::PortHandler *portHandler; // = dynamixel::PortHandler::getPortHandler(2.0);
                Port port = Port(id.baudrate, id.port);
                
                if (portHandlerPool.find(port) != portHandlerPool.end()) {
                    portHandler = portHandlerPool[port];
                } else {
                    portHandler = dynamixel::PortHandler::getPortHandler(port.port.c_str());
                    portHandler->setBaudRate(port.baudrate);
                    portHandlerPool.insert(std::make_pair(port, portHandler));
                }

                dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
                DynamixelMotor new_motor = DynamixelMotor(id.id, id.baudrate, id.port, motor.second, portHandler, packetHandler);

                this->motors.insert(std::make_pair(id, new_motor));
            }
        }
        
        /**
         * @brief A method to print all motors already loaded into the helper
         * 
         */
        void printAll() {
            for (auto& motor: motors) {
                std::cout << motor.second;
            }
            std::cout << std::endl;
        }

        /**
         * @brief Get a motor by a Motor Identifier object
         * 
         * @param identifier The motor identifier
         * @return DynamixelMotor The motor based on the motor identifier
         */
        DynamixelMotor getByMotorIdentifier(MotorIdentifier identifier) {
            return motors[identifier];
        }

        /**
         * @brief Test method, dont use
         * 
         */
        bool writePositionAsync(MotorIdentifier indentifier, double position, uint movingThreshold) {
            DynamixelMotor motor = getByMotorIdentifier(indentifier);
            auto writePos = std::async(std::launch::async, &DynamixelMotor::writePosition, &motor, position, movingThreshold);
            return writePos.get();
        }

        /**
         * @brief Set the goal of a motor in the helper
         * 
         * @param id The motor identifier
         * @param angle The desired angle of the motor
        */
        void setAngle(MotorIdentifier id, double angle) {
            std::cout << " set pos to " << thetaToPos(angle) << std::endl;

            DynamixelMotor motor = getByMotorIdentifier(id);

            motor.setGoal(thetaToPos(angle));
        }

        /**
         * @brief Set the goal of a motor in the helper
         * 
         * @param id The motor identifier
         * @param value The desired torque value
        */
        void setTorque(MotorIdentifier id, bool value) {
            DynamixelMotor motor = getByMotorIdentifier(id);
            motor.setTorque(value);
        }

};
