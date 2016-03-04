/*
 * PacketHandler.cpp
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom
 */

#include "dynamixel_sdk/PacketHandler.h"
#include "dynamixel_sdk/Protocol1PacketHandler.h"
#include "dynamixel_sdk/Protocol2PacketHandler.h"

using namespace ROBOTIS;

PacketHandler *PacketHandler::GetPacketHandler(float protocol_version)
{
    if(protocol_version == 1.0)
        return (PacketHandler *)(Protocol1PacketHandler::GetInstance());
    else if(protocol_version == 2.0)
        return (PacketHandler *)(Protocol2PacketHandler::GetInstance());

    return (PacketHandler *)(Protocol2PacketHandler::GetInstance());
}
