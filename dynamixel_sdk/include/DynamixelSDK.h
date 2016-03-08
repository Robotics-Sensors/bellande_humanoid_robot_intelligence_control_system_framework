/*
 * DynamixelSDK.h
 *
 *  Created on: 2016. 3. 8.
 *      Author: zerom
 */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_DYNAMIXELSDK_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_DYNAMIXELSDK_H_


#include "dynamixel_sdk/RobotisDef.h"
#include "dynamixel_sdk/GroupBulkRead.h"
#include "dynamixel_sdk/GroupBulkWrite.h"
#include "dynamixel_sdk/GroupSyncRead.h"
#include "dynamixel_sdk/GroupSyncWrite.h"
#include "dynamixel_sdk/Protocol1PacketHandler.h"
#include "dynamixel_sdk/Protocol2PacketHandler.h"

#ifdef __linux__
  #include "dynamixel_sdk_linux/PortHandlerLinux.h"
#endif

#if defined(_WIN32) || defined(_WIN64)
  #include "dynamixel_sdk_windows/PortHandlerWindows.h"
#endif


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_DYNAMIXELSDK_H_ */
