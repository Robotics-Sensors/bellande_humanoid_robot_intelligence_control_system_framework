/*
 * GroupSyncRead.h
 *
 *  Created on: 2016. 2. 2.
 *      Author: zerom
 */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_


#include <map>
#include <vector>
#include "RobotisDef.h"
#include "PortHandler.h"
#include "PacketHandler.h"

namespace ROBOTIS
{

class GroupSyncRead
{
private:
    PortHandler    *port_;
    PacketHandler  *ph_;

    std::vector<UINT8_T>            id_list_;
    std::map<UINT8_T, UINT8_T* >    data_list_; // <id, data>

    bool            last_result_;
    bool            is_param_changed_;

    UINT8_T        *param_;
    UINT16_T        start_address_;
    UINT16_T        data_length_;

    void    MakeParam();

public:
    GroupSyncRead(PortHandler *port, PacketHandler *ph, UINT16_T start_address, UINT16_T data_length);
    ~GroupSyncRead() { ClearParam(); }

    PortHandler     *GetPortHandler()   { return port_; }
    PacketHandler   *GetPacketHandler() { return ph_; }

    bool    AddParam    (UINT8_T id);
    void    RemoveParam (UINT8_T id);
    void    ClearParam  ();

    int     TxPacket();
    int     RxPacket();
    int     TxRxPacket();

    bool    GetData(UINT8_T id, UINT16_T address, UINT8_T *data);
    bool    GetData(UINT8_T id, UINT16_T address, UINT16_T *data);
    bool    GetData(UINT8_T id, UINT16_T address, UINT32_T *data);
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_ */
