
#include "arts_dv_tool_driver/homing/dxl_interface.hpp"

//change at 406

namespace arts_homing
{

DxlInterface::DxlInterface()
{
    m_groupSyncWritePos = NULL;
    m_groupBulkRead = NULL;
}

DxlInterface::~DxlInterface()
{

}

int DxlInterface::OpenPort(std::string portName, int baud, float protocol_version)
{
    // Set port path, protocol version
    m_portName = portName;
    m_portHandler = dynamixel::PortHandler::getPortHandler(m_portName.c_str());
    m_packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

    // Open port
    if (!m_portHandler->openPort())  {   return 0;   }

    // Set port baudrate
    m_baudrate = baud;
    if (!m_portHandler->setBaudRate(baud))  {   return 0;   }

    return 1;
}

int DxlInterface::ConfigureDevices(int numDev, const int* devIds)
{
    m_devIds.resize(numDev);
    for(int i=0; i<numDev; i++) {
        m_devIds[i] = devIds[i];
    }

    // Initialize GroupSyncWrite instance. Set up parameters to write at the same time
    m_groupSyncWritePos = std::make_unique<dynamixel::GroupSyncWrite>(m_portHandler, m_packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    m_groupSyncWriteVel = std::make_unique<dynamixel::GroupSyncWrite>(m_portHandler, m_packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
    
    // Initialize GroupBulkRead instance. Set up parameters to bulk read
    m_groupBulkRead = std::make_unique<dynamixel::GroupBulkRead>(m_portHandler, m_packetHandler);
    for(int i=0; i<m_devIds.size(); i++)
    {
        // We will read present position of each actuators
        bool dxl_addparam_result = m_groupBulkRead->addParam(m_devIds[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%0m_devIds.size()d] grouBulkRead addparam failed", m_devIds[i]);
            return 0;
        }
    }
    return 1;
}

int DxlInterface::EnableTorque()
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    for(int i=0; i<m_devIds.size(); i++)
    {
        dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", m_devIds[i]);
        }
    }
    return 1;
}

int DxlInterface::DisableTorque()
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    for(int i=0; i<m_devIds.size(); i++)
    {
        dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
        {
            printf("Dynamixel#%d has been successfully disconnected \n", m_devIds[i]);
        }
    }
    return 1;
}

void DxlInterface::ClosePort()
{
    // if(m_groupSyncWritePos)    delete m_groupSyncWritePos;
    // if(m_groupBulkRead)    delete m_groupBulkRead;

    // Close port
    m_portHandler->closePort();

}

bool DxlInterface::GetBulkPos(int *pPos)
{
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	bool dxl_getdata_result = false;                 // GetParam result

	// Read current position. Bulkread is synchronized read
	dxl_comm_result = m_groupBulkRead->txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		fprintf(stderr,"Sparta!! ");
		fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
		return 0;
	}

	for(int i=0; i<m_devIds.size(); i++)
	{
		dxl_getdata_result = m_groupBulkRead->isAvailable(m_devIds[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
		  fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", m_devIds[i]);
		  return 0;
		}

		// Get Dynamixel present position value
		pPos[i] = m_groupBulkRead->getData(m_devIds[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
		//printf("%d  ",dxl_curr_position[i]);
	}
	//printf("\n ");

	return 1;

}

bool DxlInterface::SetBulkPos(const int *pPos)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    uint8_t param_goal_position[LEN_GOAL_POSITION];

    // Fill in data to write
    for(int i=0; i<m_devIds.size(); i++)
    {
        // Allocate goal position value into byte array
        // param_goal_position[0] = DXL_LOBYTE(pPos[i]);
        // param_goal_position[1] = DXL_HIBYTE(pPos[i]);
        for(int j=0; j<LEN_GOAL_POSITION; j++)  {
            param_goal_position[j] = static_cast<char>( (pPos[i] >> (8*j)) & 0xFF );
        }

        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = m_groupSyncWritePos->addParam(m_devIds[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_devIds[i]);
            return 0;
        }
    }

    // Synchronized write to multiple dynamixel
	dxl_comm_result = m_groupSyncWritePos->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }

    // Clear syncwrite parameter storage
    m_groupSyncWritePos->clearParam();

    return 1;
}

bool DxlInterface::SetBulkVel(const int *pVel)
{
    // Wheel Mode It is a moving speed to Goal direction.
    // 0~2047 (0X7FF) can be used, and the unit is about 0.114rpm.
    // If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.
    // If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction.
    // That is, the 10th bit becomes the direction bit to control the direction.


    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    uint8_t param_move_speed[LEN_GOAL_VELOCITY];

    // Fill in data to write
    for(int i=0; i<m_devIds.size(); i++)
    {
        // Allocate goal position value into byte array
        // param_move_speed[0] = DXL_LOBYTE(pVel[i]);
        // param_move_speed[1] = DXL_HIBYTE(pVel[i]);
        for(int j=0; j<LEN_GOAL_VELOCITY; j++)  {
            param_move_speed[j] = static_cast<char>( (pVel[i] >> (8*j)) & 0xFF );
        }

        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = m_groupSyncWriteVel->addParam(m_devIds[i], param_move_speed);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_devIds[i]);
            return 0;
        }
    }

    // Synchronized write to multiple dynamixel
	dxl_comm_result = m_groupSyncWriteVel->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }

    // Clear syncwrite parameter storage
    m_groupSyncWriteVel->clearParam();

    return 1;
}

bool DxlInterface::SetOperatingMode(int mode)
{
    // Set both CW and CCW limit to 0 to enable Wheel Mode (infinite turn)
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    for(int i=0; i<m_devIds.size(); i++)
    {
        dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[i], ADDR_OPMODE, mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
        {
            printf("Dynamixel#%d has been set to mode %d", m_devIds[i], mode);
        }
    }

    return 1;
}

//to set the speed of the motors from the beginning to be used in position control
bool DxlInterface::SetProfileVelocity(int velocity)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    for(size_t i = 0; i < m_devIds.size(); i++)
    {
        dxl_comm_result = m_packetHandler->write4ByteTxRx(m_portHandler, m_devIds[i], ADDR_PROFILE_VELOCITY, velocity, &dxl_error);
        
        if (dxl_comm_result != COMM_SUCCESS)
        {
            return false;
        }
    }
    return true;
}

bool DxlInterface::DisableSingleTorque(uint8_t id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    return dxl_comm_result == COMM_SUCCESS;
}

bool DxlInterface::EnableSingleTorque(uint8_t id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    return dxl_comm_result == COMM_SUCCESS;
}



// bool DxlInterface::SetWheelMode(bool onoff)
// {
//     // Set both CW and CCW limit to 0 to enable Wheel Mode (infinite turn)
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     uint8_t dxl_error = 0;                          // Dynamixel error
//     uint16_t flag = 0;

//     for(int i=0; i<m_devIds.size(); i++)
//     {
//         if(onoff)   {   flag = 0;   }
//         dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, m_devIds[i], ADDR_CW_ANGLE_LIM, flag, &dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             // printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
//             return 0;
//         }
//         else if (dxl_error != 0)
//         {
//             // printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
//             return 0;
//         }
//         else
//         {
//             // printf("Dynamixel#%d has been set to ", m_devIds[i]);
//             // if(onoff)   {   printf("MultiTurn mode\n"); }
//             // else        {   printf("Joint mode\n"); }
//         }

//         if(onoff)   {   flag = 0;       }
//         else        {   flag = 4095;    }
//         dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, m_devIds[i], ADDR_CCW_ANGLE_LIM, flag, &dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             // printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
//             return 0;
//         }
//         else if (dxl_error != 0)
//         {
//             // printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
//             return 0;
//         }
//         else
//         {
//             // printf("Dynamixel#%d has been set to ", m_devIds[i]);
//             // if(onoff)   {   printf("Wheel mode\n"); }
//             // else        {   printf("Joint mode\n"); }
//         }

//     }

//     return 1;
// }

// bool DxlInterface::SetGain(int id, const float* DIP)
// {
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     uint8_t dxl_error = 0;                          // Dynamixel error
//     uint8_t dipGain[m_devIds.size()];

//     // Conversion formula from Dynamixel manual
//     dipGain[0] = DIP[0]*250.0;
//     dipGain[1] = DIP[1]*2.048;
//     dipGain[2] = DIP[2]*8.0;

//     for(int i=0;i<m_devIds.size(); i++)
//     {
//         dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[id], ADDR_DIP_GAIN+i, dipGain[i],&dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             // printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
//             return 0;
//         }
//         else if (dxl_error != 0)
//         {
//             // printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
//             return 0;
//         }
//         else
//         {
//             // printf("DIP[%d] of Dynamixel %d set\n",i,id);
//         }
//     }
//     return 1;
// }


int32_t DxlInterface::ReadPosition(int id)
{
    uint32_t pos_data = 0;
    uint8_t dxl_error = 0;
   
    int dxl_comm_result = m_packetHandler->read4ByteTxRx(m_portHandler, id, ADDR_PRESENT_POSITION, &pos_data, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
    }
    
    //convert to int32 to deal with negatives
    return static_cast<int32_t>(pos_data);
}

int16_t DxlInterface::ReadCurrent(int id)
{
    uint16_t cur_data = 0;
    uint8_t dxl_error = 0;
    
    int dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_PRESENT_CURRENT, &cur_data, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
    }
    
    //convert to int16 to deal with negatives
    return static_cast<int16_t>(cur_data);
}

bool DxlInterface::ReadState(int id, int32_t& pos, int32_t& vel, int16_t& cur)
{
    // Control table 126..135: Present Current (2), Present Velocity (4), Present Position (4)
    uint8_t data[10] = {0};
    uint8_t dxl_error = 0;
    int dxl_comm_result = m_packetHandler->readTxRx(m_portHandler, id, ADDR_PRESENT_CURRENT, 10, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        return false;
    }
    cur = static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
    uint32_t v = static_cast<uint32_t>(data[2]) | (static_cast<uint32_t>(data[3]) << 8) |
                 (static_cast<uint32_t>(data[4]) << 16) | (static_cast<uint32_t>(data[5]) << 24);
    uint32_t p = static_cast<uint32_t>(data[6]) | (static_cast<uint32_t>(data[7]) << 8) |
                 (static_cast<uint32_t>(data[8]) << 16) | (static_cast<uint32_t>(data[9]) << 24);
    vel = static_cast<int32_t>(v);
    pos = static_cast<int32_t>(p);
    return true;
}

bool DxlInterface::SyncReadState(int32_t* pos, int32_t* vel, int16_t* cur)
{
    if (!m_groupSyncReadState) {
        m_groupSyncReadState = std::make_unique<dynamixel::GroupSyncRead>(m_portHandler, m_packetHandler,
                                                                          ADDR_PRESENT_CURRENT, 10);
        for (int id : m_devIds) {
            if (!m_groupSyncReadState->addParam(id)) { m_groupSyncReadState.reset(); return false; }
        }
    }
    if (m_groupSyncReadState->txRxPacket() != COMM_SUCCESS) return false;
    for (size_t i = 0; i < m_devIds.size(); ++i) {
        int id = m_devIds[i];
        if (!m_groupSyncReadState->isAvailable(id, ADDR_PRESENT_CURRENT, 10)) return false;
        cur[i] = static_cast<int16_t>(m_groupSyncReadState->getData(id, ADDR_PRESENT_CURRENT, 2));
        vel[i] = static_cast<int32_t>(m_groupSyncReadState->getData(id, 128, 4));
        pos[i] = static_cast<int32_t>(m_groupSyncReadState->getData(id, ADDR_PRESENT_POSITION, 4));
    }
    return true;
}

int DxlInterface::ReadHardwareError(int id)
{
    uint8_t err = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = m_packetHandler->read1ByteTxRx(m_portHandler, id, 70, &err, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        return -1;
    }
    return static_cast<int>(err);
}

bool DxlInterface::SetProfileVelocityId(int id, int velocity)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = m_packetHandler->write4ByteTxRx(m_portHandler, id, ADDR_PROFILE_VELOCITY,
                                                          static_cast<uint32_t>(velocity), &dxl_error);
    return dxl_comm_result == COMM_SUCCESS;
}

bool DxlInterface::SetGoalCurrent(int id, uint16_t current)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_GOAL_CURRENT, current, &dxl_error);
        
    if (dxl_comm_result != COMM_SUCCESS)
    {
        return false;
    }
    
    return true;
}


}

// int DxlInterface::SetOffset()
// {
//     int dxl_curr_position[m_devIds.size()] = {0,0,0};  // Current position

//     // Read position
//     bool res = GetBulkPos(dxl_curr_position);
//     if(!res)    {
//         fprintf(stderr, "Error while reading position");
//         return 0;
//     }

//     for(int i=0; i<m_devIds.size(); i++)  {   m_MtrCntOffset[i] = dxl_curr_position[i];   }
//     printf("Offset is %d %d %d\n", m_MtrCntOffset[0],m_MtrCntOffset[1],m_MtrCntOffset[2]);
//     for(int i=0; i<m_devIds.size(); i++)  {   m_TgtMtrCnt[i] = m_MtrCntOffset[i];   }
// }

// void* DxlInterface::ThrCallback(void* pData)
// {
// 	DxlInterface* pDriver = (DxlInterface*)pData;

//     bool res;

//     int dxl_goal_position[m_devIds.size()] = {0,0,0};  // Goal position
//     int dxl_curr_position[m_devIds.size()] = {0,0,0};  // Current position


//     //    // Data variables
//     //    int tgtPos[m_devIds.size()];

// 	// RT variables
// 	long long int time, time_r;
// 	struct timespec t, t_real;
// 	struct sched_param param;
// 	ssize_t ret = 0;

// 	int errCnt = 0;

// 	// Increase priority (this thread will preempt the kernel functions)
// 	param.sched_priority = MY_PRIORITY;
// 	if(sched_setscheduler(0, SCHED_FIFO, &param) != 0)
// 	{
// 		perror("sched_setscheduler failed");
// 		return 0;//exit(-1);
// 	}

// 	// Set clock
// 	clock_gettime(CLOCK_REALTIME ,&t); /* start after one second */
// 	t.tv_nsec += 100*MS;
// 	tsnorm(&t);


//     while(ros::ok())
//     {
// 		// Update dxl_goal_position and save current position
// 		pthread_mutex_lock( &mutex1 );
// 		for(int i=0; i<m_devIds.size(); i++)  {
// 			dxl_goal_position[i] = pDriver->m_TgtMtrCnt[i];
// 			//pDriver->m_CurrMtrCnt[i] = pDriver->m_TgtMtrCnt[i];
// 			pDriver->m_CurrMtrCnt[i] = dxl_curr_position[i];
// 		}
// 		pthread_mutex_unlock( &mutex1 );

// 		// Command position
// 		res = pDriver->SetJpos(dxl_goal_position);
// 		if(!res)    {
// 			fprintf(stderr, "Error while commanding position %d\n", errCnt);
// 			//return 0;
// 			errCnt++;
// 		}

// 		// Read position
// 		res = pDriver->GetJpos(dxl_curr_position);
// 		if(!res)    {
// 			fprintf(stderr, "Error while reading position %d\n", errCnt);
// 			//return 0;
// 			errCnt++;
// 		}


// 		// Check if loop ran within time
// 		t.tv_nsec += INTERVAL;
// 		tsnorm(&t);

// 		clock_gettime(CLOCK_REALTIME ,&t_real);
// 		time_r = (long long int)t_real.tv_nsec + (long long int)t_real.tv_sec*NSEC_PER_SEC;
// 		time  = (long long int)t.tv_nsec + (long long int)t.tv_sec*NSEC_PER_SEC;

// 		if(time < time_r){
// 			//_this->device_state = 0;
// 			//ROS_INFO("Failed to meet time");
// 			//return 0;
// 		}

// 		// Sleep for remaining time of the loop
// 		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t, NULL);


// 		//usleep(10000);

//       }

//     // Return Motor to original location at the end
//     for(int i=0; i<m_devIds.size(); i++)  {   dxl_goal_position[i] = pDriver->m_MtrCntOffset[i];   }
//     //for(int i=0; i<m_devIds.size(); i++)  {   dxl_goal_position[i] = -100;   }
//     pDriver->SetJpos(dxl_goal_position);
//     sleep(m_devIds.size());

//     return 0;
// }

