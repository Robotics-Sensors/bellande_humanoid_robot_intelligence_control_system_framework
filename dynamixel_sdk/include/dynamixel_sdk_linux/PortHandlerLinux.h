/*
 * PortHandlerLinux.h
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_H_
#define ROBOTIS_FRAMEWORK_DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_H_


#include "dynamixel_sdk/PortHandler.h"

namespace ROBOTIS
{

class PortHandlerLinux : public PortHandler
{
private:
    int     socket_fd_;
    int     baudrate_;
    char    port_name_[30];

    double  packet_start_time_;
    double  packet_timeout_;
    double  tx_time_per_byte;

    bool    SetupPort(const int cflag_baud);
    bool    SetCustomBaudrate(int speed);
    int     GetCFlagBaud(const int baudrate);

    double  GetCurrentTime();
    double  GetTimeSinceStart();

public:
    PortHandlerLinux(const char *port_name);
    virtual ~PortHandlerLinux() { ClosePort(); }

    bool    OpenPort();
    void    ClosePort();
    void    ClearPort();

    void    SetPortName(const char *port_name);
    char   *GetPortName();

    bool    SetBaudRate(const int baudrate);
    int     GetBaudRate();

    int     GetBytesAvailable();

    int     ReadPort(UINT8_T *packet, int length);
    int     WritePort(UINT8_T *packet, int length);

    void    SetPacketTimeout(UINT16_T packet_length);
    void    SetPacketTimeout(double msec);
    bool    IsPacketTimeout();
};

}


#endif /* ROBOTIS_FRAMEWORK_DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_H_ */
