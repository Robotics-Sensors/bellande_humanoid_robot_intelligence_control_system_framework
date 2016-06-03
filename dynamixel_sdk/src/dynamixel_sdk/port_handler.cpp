/*
 * PortHandler.cpp
 *
 *  Created on: 2016. 2. 5.
 *      Author: zerom, leon
 */
#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "dynamixel_sdk/PortHandler.h"

#ifdef __linux__
  #include "dynamixel_sdk_linux/PortHandlerLinux.h"
#endif

#if defined(_WIN32) || defined(_WIN64)
  #include "dynamixel_sdk_windows/PortHandlerWindows.h"
#endif

using namespace ROBOTIS;

PortHandler *PortHandler::GetPortHandler(const char *port_name)
{
#ifdef __linux__
    return (PortHandler *)(new PortHandlerLinux(port_name));
#endif

#if defined(_WIN32) || defined(_WIN64)
    return (PortHandler *)(new PortHandlerWindows(port_name));
#endif
}
