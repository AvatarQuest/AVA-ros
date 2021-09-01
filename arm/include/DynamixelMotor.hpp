#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <iostream>
#include <string>
#include <future>

#include "dynamixel_sdk/dynamixel_sdk.h"                              // Uses Dynamixel SDK library

/**
 * @brief A structure to deal with address table values
 * 
 */
typedef struct AddressTableBase {
    typedef uint address;
    /**
     * @brief The address for enabling the torque of the motor
     * 
     */
    address TORQUE_ENABLE;
    /**
     * @brief The address for setting the goal position of the motor
     * 
     */
    address GOAL_POSITION;
    /**
     * @brief The address for getting the present postion of the motor
     * 
     */
    address PRESENT_POSITION;

    AddressTableBase() {};
} AddressTableBase;

typedef struct XM430W350T_TABLE: AddressTableBase {
    XM430W350T_TABLE() {
        TORQUE_ENABLE = 64;
        GOAL_POSITION = 116;
        PRESENT_POSITION = 132;
    }
} XM430W350T_TABLE;


/**
 * @brief A class to easily control a single dynamixel motor
 */
class DynamixelMotor {
    private:
        uint id, baudrate;
        std::string port;

    protected:
        bool checkCommResult(int result, uint8_t error) {
            if (result != COMM_SUCCESS) {
                std::string output = packetHandler->getTxRxResult(result);
                
                std::cout << output << std::endl;
                
                if (output.find("Failed transmit instruction packet!") != std::string::npos) {
                    std::cout << "You might be seeing this error because there is no power to the motor or another application is using the port (port busy)" << std::endl;
                }
                return false;
            } else if (error != 0) {
                std::cout << "ERR for ID: " << id << " -> "  << packetHandler->getRxPacketError(error) << std::endl;
                return false;
            } else {
                return true;
            }
        }

    public:
        /**
         * The address table with memory location values
         */
        AddressTableBase addressTable;
        /**
         * The port handler to the corresponding port
         */
        dynamixel::PortHandler *portHandler;
        /**
         * The packet handler for the corresponding baudrate
         */
        dynamixel::PacketHandler *packetHandler;
        
        /**
         * @brief Construct a new Dynamixel Motor object
         * 
         * @param id The id on the motor
         * @param baudrate The baudrate of the motor
         * @param port The port of the motor
         * @param addressTable The address table of the specific model of the dynamixel containing the memory values
         */
        DynamixelMotor(uint id=0, uint baudrate=0, std::string port=nullptr, AddressTableBase addressTable=AddressTableBase()) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;
            this->addressTable = addressTable;

            this->portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
            this->packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
        }

        /**
         * @brief Construct a new Dynamixel Motor object with packet and port handler injection
         * 
         * @param id The integer id of the dynamixel
         * @param baudrate The integer baudrate of the dynamixel
         * @param port The port path as a string 
         * @param portHandler The dynamixel port handler pointer
         * @param packetHandler The dynamixel packet handler pointer
         */
        DynamixelMotor(uint id, uint baudrate, std::string port, AddressTableBase addressTable, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;
            this->addressTable = addressTable;

            this->portHandler = portHandler;
            this->packetHandler = packetHandler;
        }

        friend std::ostream & operator << (std::ostream &out, const DynamixelMotor &motor);

        /**
         * @brief Get the id 
         * 
         * @return int 
         */
        int getId() {
            return id;
        }

        /**
         * @brief Get the baudrate 
         * 
         * @return int 
         */
        int getBaudrate() {
            return baudrate;
        }
        
        /**
         * @brief Get the port 
         * 
         * @return std::string 
         */
        std::string getPort() {
            return port;
        }

        /**
         * @brief A method to set up the inital port and packet handler. No need to run this if port and packet handler pointers were already given in the constructor
         * 
         * @return true The initialization succeeded
         * @return false The initialization failed
         */
        bool init() {
            if (portHandler->openPort()) {
                std::cout << "Opened port " << port << " succesfully" << std::endl;
            } else {
                std::cout << "Failed to open port " << port << std::endl;
                std::cout << "Did you run \"sudo chmod a+rw " << port << "\"? Also check if the port is being used by another program." << std::endl;
                return false;
            }
 
            if (portHandler->setBaudRate(baudrate)) {
                std::cout << "Set baudrate to " << baudrate << " succesfully" << std::endl;
            } else {
                close();
                std::cout << "Failed to set baudrate to " << baudrate << std::endl;
                return false;
            }

            return true;
        }

        /**
         * @brief Pings the dynamixel
         * 
         * @return true The ping succeeded
         * @return false The ping failed
         */
        bool ping() {
            uint8_t error = 0;                          
            uint16_t model_number;

            int result = packetHandler->ping(portHandler, id, &model_number, &error);
            bool success = checkCommResult(result, error);
            if (success) {
                std::cout << "[ID:" << id <<"] ping Succeeded" << std::endl;
                return true;
            }
            return false;
        }
        
        /**
         * @brief Reads the current position of the dynamixel
         * 
         * @return double The value in dxl units of the present position of the servo
         */
        int32_t readPosition() {
            int32_t position;
            uint8_t error = 0;

            int result = packetHandler->read4ByteTxRx(portHandler, id, addressTable.PRESENT_POSITION, (uint32_t*)&position, &error);
            bool success = checkCommResult(result, error);
            
            return success ? position : 0;
        } 

        /**
         * @brief 
         * 
         * @param position The position in dynamixel units
         * @param movingThreshold The margin of error in the movement 
         * @return true Moving succeded
         * @return false Moving failed
         */
        bool writePosition(double position, uint movingThreshold) {
            uint8_t error = 0;
            int result = packetHandler->write4ByteTxRx(portHandler, id, addressTable.GOAL_POSITION, position, &error);

            if (!checkCommResult(result, error)) {
                return false;
            }
            
            int32_t current_position = 0;
            do {
                current_position = readPosition();
            } while (abs(position - current_position) > movingThreshold); 

            return true;
        }

        bool setGoal(double position) {
           uint8_t error = 0;
           int result = packetHandler->write4ByteTxRx(portHandler, id, addressTable.GOAL_POSITION, position, &error);
           
           return checkCommResult(result, error);
        }

        /**
         * @brief Set the value of torque
         * 
         * @param value The desired torque
         * @return true The write succeeded
         * @return false The write failed
         */
        bool setTorque(bool value) {
            uint8_t error;
            int result = packetHandler->write1ByteTxRx(portHandler, id, addressTable.TORQUE_ENABLE, value, &error);
            return checkCommResult(result, error);
        }

        /**
         * @brief Closes the port for the corrosponsong motor
         */
        void close() {
            portHandler->closePort();
        }

        bool reboot() {
            uint8_t error;
            int result = packetHandler->reboot(portHandler, id, &error);
            return checkCommResult(result, error);
        }
};

std::ostream & operator << (std::ostream &out, DynamixelMotor &motor) {
    out << "Dynamixel Motor - ID: " << motor.getId() << ", Baudrate: " << motor.getBaudrate() << ", Port: " << motor.getPort(); 
    return out; 
} 

// int main() {
//     AddressTableBase table = XM430W350T_TABLE();

//     DynamixelMotor motor = DynamixelMotor(12, 57600, "/dev/tty.usbserial-FT4TCRQV", table);
//     if (!motor.init()) {
//         exit(1);
//     }

//     std::cout << motor;

//     motor.ping();

//     std::cout << "position: " << motor.readPosition() << std::endl;

//     motor.setTorque(true);
//     std::cout << motor.writePosition(2530, 10) << std::endl;

//     std::cout << "position: " << motor.readPosition() << std::endl;
//     motor.setTorque(false);

//     motor.close();
// }