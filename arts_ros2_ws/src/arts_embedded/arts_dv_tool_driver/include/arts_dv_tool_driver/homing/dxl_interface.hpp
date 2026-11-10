// -------------------------------------------------------------- //
// Interface to dynamixel using dynamixel_sdk
//
// Last Update : 20251219
//
// -------------------------------------------------------------- //

//changes at 47,140

#ifndef DXL_INTERFACE_H
#define DXL_INTERFACE_H

//#include "arts_homing/visibility_control.h"

#include <string>
#include <vector>
#include <memory>

// headers for dynamixel_sdk
#include <dynamixel_sdk/dynamixel_sdk.h>		 // Uses Dynamixel SDK library that is installed with the ROS


// Control table address and byte length for Dynamixel XC330-M288
#define ADDR_GOAL_POSITION          116 // Between min/max position limit stored in address 52/48, unit: encoder pulse
#define LEN_GOAL_POSITION           4

#define ADDR_GOAL_VELOCITY          104 // Between -/+ velocity limit stored in address 44, unit: 0.229 rev/min
#define LEN_GOAL_VELOCITY           4

#define ADDR_PRESENT_POSITION       132 // unit: encoder pulse
#define LEN_PRESENT_POSITION        4   // unit: encoder pulse

#define ADDR_TORQUE_ENABLE          64  // Torque enable flag
#define LEN_TORQUE_ENABLE           1

#define ADDR_OPMODE                 11 
#define LEN_OPMODE                  1
#define OPMODE_CURRENT_CTRL         0
#define OPMODE_VELOCITY_CTRL        1   // Formerly wheel mode
#define OPMODE_POSITION_CTRL        3
#define OPMODE_EXTPOS_CTRL          4   // Extended position control (multi turn)
#define OPMODE_POSCURR_CTRL         5   // Current based position control
#define OPMODE_PWM_CTRL             16

#define ADDR_PRESENT_CURRENT        126
#define LEN_PRESENT_CURRENT         2

//CHANGE
#define ADDR_GOAL_CURRENT	    102
#define LEN_GOAL_CURRENT            2

#define ADDR_PROFILE_VELOCITY       112      
#define LEN_PROFILE_VELOCITY        4

// // // Control table address
// // #define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
// #define ADDR_MX_GOAL_POSITION           30
// // #define ADDR_MX_PRESENT_POSITION        36
// #define ADDR_CW_ANGLE_LIM               6                   // Set this to 4095 to enable multiturn
// #define ADDR_CCW_ANGLE_LIM              8                   // Set this to 4095 to enable multiturn
// // #define ADDR_ANGLE_DIVIDER              22
// #define ADDR_DIP_GAIN                   26
// // #define ADDR_MOVING_SPEED               32

// // // Data Byte Length
// #define LEN_MX_GOAL_POSITION            2
// // #define LEN_MX_PRESENT_POSITION         2
// #define LEN_CW_ANGLE_LIM                2
// // #define LEN_ANGLE_DIVIDER               1
// #define LEN_DIP_GAIN                    3                   // 1 byte per each D,I,P Gain
// // #define LEN_MOVING_SPEED                2


// //X-Series Control Table Addresses
// #define ADDR_MX_TORQUE_ENABLE           64
// #define ADDR_OPERATING_MODE             11
// #define ADDR_MOVING_SPEED               104 //Goal Velocity in X-Series
// #define ADDR_MX_PRESENT_POSITION        132
// #define ADDR_PRESENT_CURRENT            126

// //X-Series Data Byte Lengths
// #define LEN_MX_TORQUE_ENABLE            1
// #define LEN_OPERATING_MODE              1
// #define LEN_MOVING_SPEED                4   //X-Series uses 4 bytes for velocity
// #define LEN_MX_PRESENT_POSITION         4
// #define LEN_PRESENT_CURRENT             2



// Protocol version
// #define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
// #define BAUDRATE                        1000000

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MULTITURN_ENABLE                4095

// tty means TeleTYpewriter, which used to be the input output 'terminals'
// of the computers system in the past. Now, it refers to the general 'terminals' of the device.
// Change this definition for the correct port. We will be using terminal of the COM port.
//#define MODEMDEVICE "/dev/ttyUSB0"		// This is for FTDI USB RS485 cable

namespace arts_homing
{

class DxlInterface
{

public:
	DxlInterface();
	~DxlInterface();

    /// Initialize Dynamixel, Open and close communication. 
    // Protocol version is 1.0 or 2.0, returns 1 on success    
    int OpenPort(std::string portName, int baud, float protocol_version = 2.0);
    int ConfigureDevices(int numDev, const int* devIds);
    void ClosePort();

    /// Enable, disable torque. returns 1 on success
    int EnableTorque();
    int DisableTorque();

    /// Read and write motor commands
    bool GetBulkPos(int* pPos);    // in counts
    bool SetBulkPos(const int* pPos);
    // bool GetBulkVel(int* pPos);
    bool SetBulkVel(const int* pVel);
    
    /// Configure motors
    // bool SetWheelMode(bool onoff);
    bool SetOperatingMode(int mode);
    // bool SetGain(int id, const float* DIP);
    // int SetOffset();

    // Added by the adapter designer, 2026-07-21
    int32_t ReadPosition(int id);
    int16_t ReadCurrent(int id);
    bool SetProfileVelocity(int velocity);
    //CHANGE
    bool SetGoalCurrent(int id, uint16_t current);
    bool DisableSingleTorque(uint8_t id);
    bool EnableSingleTorque(uint8_t id);

    // Added for jaw homing (Nov 2026)
    // Reads present current, velocity and position of ONE motor in a single transaction.
    bool ReadState(int id, int32_t& pos, int32_t& vel, int16_t& cur);
    // Hardware Error Status (address 70). Returns -1 on communication failure.
    int ReadHardwareError(int id);
    // Profile velocity of ONE motor (SetProfileVelocity() writes all motors).
    bool SetProfileVelocityId(int id, int velocity);
    // ONE transaction for all configured motors: present current, velocity, position.
    // Arrays are indexed like the configured device list (ids 1..5 -> index 0..4).
    bool SyncReadState(int32_t* pos, int32_t* vel, int16_t* cur);


private:

    // Dynamixel Variables. Class for handling serial port, handling packets
    dynamixel::PortHandler*     m_portHandler;
    dynamixel::PacketHandler*   m_packetHandler;
    std::unique_ptr<dynamixel::GroupSyncWrite>  m_groupSyncWritePos;
    std::unique_ptr<dynamixel::GroupSyncWrite>  m_groupSyncWriteVel;
    std::unique_ptr<dynamixel::GroupBulkRead>   m_groupBulkRead;
    std::unique_ptr<dynamixel::GroupSyncRead>   m_groupSyncReadState;

    // Portname, Dynamixel Device IDs, baud rates
    std::string m_portName;
    int m_baudrate;
    std::vector<int> m_devIds;
};

}

#endif




// // These header files and definitions are for serial communications
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>	// 'file control'
// #include <termios.h>
// #include <stdio.h>
// #include <signal.h>		// For catching 'ctrl-c' input
// #include <sys/ioctl.h>
// #include <linux/serial.h>

// ////////////////////////////start rt prempt include///////////////////////////////
// #include <time.h>
// #include <sched.h>
// #include <sys/mman.h>
// #include <string.h>
// #include <unistd.h>
// #include <malloc.h>
// #include <pthread.h>
// ////////////////////////////end rt prempt include///////////////////////////////

// #define _POSIX_SOURCE 1 /* POSIX compliant source */

// #define MAXLOOF 50

// ///////////////////////////start rt prempt define///////////////////////////////
// #define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50 as the priority of kernel tasklets and interrupt handler by default */
// #define POOLSIZE (1*1024*1024) /* The maximum stack size which is guaranteed safe to access without faulting */
// #define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */
// #define N_EVER 100

// #define NS  	(1)
// #define US  	(1000 * NS)
// #define MS  	(1000 * US)
// #define SEC 	(1000 * MS)
// ////////////////////////////end rt prempt define///////////////////////////////
