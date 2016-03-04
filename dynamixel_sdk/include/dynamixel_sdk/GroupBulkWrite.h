/*
 * GroupBulkWrite.h
 *
 *  Created on: 2016. 2. 2.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_
#define ROBOTIS_FRAMEWORK_DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_


#include <map>
#include <vector>
#include "RobotisDef.h"
#include "PortHandler.h"
#include "PacketHandler.h"

namespace ROBOTIS
{

class GroupBulkWrite
{
private:
    PortHandler    *port_;
    PacketHandler  *ph_;

    std::vector<UINT8_T>            id_list_;
    std::map<UINT8_T, UINT16_T>     address_list_;  // <id, start_address>
    std::map<UINT8_T, UINT16_T>     length_list_;   // <id, data_length>
    std::map<UINT8_T, UINT8_T *>    data_list_;     // <id, data>

    UINT8_T        *param_;
    UINT16_T        param_length_;

    void    MakeParam();

public:
    GroupBulkWrite(PortHandler *port, PacketHandler *ph);
    ~GroupBulkWrite() { ClearParam(); }

    PortHandler     *GetPortHandler()   { return port_; }
    PacketHandler   *GetPacketHandler() { return ph_; }

    bool    AddParam    (UINT8_T id, UINT16_T start_address, UINT16_T data_length, UINT8_T *data);
    void    RemoveParam (UINT8_T id);
    bool    ChangeParam (UINT8_T id, UINT16_T start_address, UINT16_T data_length, UINT8_T *data);
    void    ClearParam  ();

    int     TxPacket();
};

}


#endif /* ROBOTIS_FRAMEWORK_DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPBULKWRITE_H_ */
