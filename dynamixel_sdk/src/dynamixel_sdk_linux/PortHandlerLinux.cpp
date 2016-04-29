/*
 * PortHandlerLinux.cpp
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom, leon
 */

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "dynamixel_sdk_linux/PortHandlerLinux.h"

#define LATENCY_TIMER   4  // msec (USB latency timer)

using namespace ROBOTIS;

PortHandlerLinux::PortHandlerLinux(const char *port_name)
    : socket_fd_(-1),
      baudrate_(DEFAULT_BAUDRATE),
      packet_start_time_(0.0),
      packet_timeout_(0.0),
      tx_time_per_byte(0.0)
{
    is_using = false;
    SetPortName(port_name);
}

bool PortHandlerLinux::OpenPort()
{
    return SetBaudRate(baudrate_);
}

void PortHandlerLinux::ClosePort()
{
    if(socket_fd_ != -1)
        close(socket_fd_);
    socket_fd_ = -1;
}

void PortHandlerLinux::ClearPort()
{
    tcflush(socket_fd_, TCIOFLUSH);
}

void PortHandlerLinux::SetPortName(const char *port_name)
{
    strcpy(port_name_, port_name);
}

char *PortHandlerLinux::GetPortName()
{
    return port_name_;
}

// TODO: baud number ??
bool PortHandlerLinux::SetBaudRate(const int baudrate)
{
    int _baud = GetCFlagBaud(baudrate);

    ClosePort();

    if(_baud <= 0)   // custom baudrate
    {
        SetupPort(B38400);
        baudrate_ = baudrate;
        return SetCustomBaudrate(baudrate);
    }
    else
    {
        baudrate_ = baudrate;
        return SetupPort(_baud);
    }
}

int PortHandlerLinux::GetBaudRate()
{
    return baudrate_;
}

int PortHandlerLinux::GetBytesAvailable()
{
    int _bytes_available;
    ioctl(socket_fd_, FIONREAD, &_bytes_available);
    return _bytes_available;
}

int PortHandlerLinux::ReadPort(UINT8_T *packet, int length)
{
    return read(socket_fd_, packet, length);
}

int PortHandlerLinux::WritePort(UINT8_T *packet, int length)
{
    return write(socket_fd_, packet, length);
}

void PortHandlerLinux::SetPacketTimeout(UINT16_T packet_length)
{
    packet_start_time_  = GetCurrentTime();
    packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerLinux::SetPacketTimeout(double msec)
{
    packet_start_time_  = GetCurrentTime();
    packet_timeout_     = msec;
}

bool PortHandlerLinux::IsPacketTimeout()
{
    if(GetTimeSinceStart() > packet_timeout_)
    {
        packet_timeout_ = 0;
        return true;
    }
    return false;
}

double PortHandlerLinux::GetCurrentTime()
{
	struct timespec _tv;
	clock_gettime( CLOCK_REALTIME, &_tv);
	return ((double)_tv.tv_sec*1000.0 + (double)_tv.tv_nsec*0.001*0.001);
}

double PortHandlerLinux::GetTimeSinceStart()
{
    double _time;

    _time = GetCurrentTime() - packet_start_time_;
    if(_time < 0.0)
        packet_start_time_ = GetCurrentTime();

    return _time;
}

bool PortHandlerLinux::SetupPort(int cflag_baud)
{
    struct termios newtio;

    socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(socket_fd_ < 0)
    {
        printf("[PortHandlerLinux::SetupPort] Error opening serial port!\n");
        return false;
    }

    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;

    // clean the buffer and activate the settings for the port
    tcflush(socket_fd_, TCIFLUSH);
    tcsetattr(socket_fd_, TCSANOW, &newtio);

    tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
    return true;
}

bool PortHandlerLinux::SetCustomBaudrate(int speed)
{
    // try to set a custom divisor
    struct serial_struct ss;
    if(ioctl(socket_fd_, TIOCGSERIAL, &ss) != 0)
    {
        printf("[PortHandlerLinux::SetCustomBaudrate] TIOCGSERIAL failed!\n");
        return false;
    }

    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
    int closest_speed = ss.baud_base / ss.custom_divisor;

    if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
    {
        printf("[PortHandlerLinux::SetCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
        return false;
    }

    if(ioctl(socket_fd_, TIOCSSERIAL, &ss) < 0)
    {
        printf("[PortHandlerLinux::SetCustomBaudrate] TIOCSSERIAL failed!\n");
        return false;
    }

    tx_time_per_byte = (1000.0 / (double)speed) * 10.0;
    return true;
}

int PortHandlerLinux::GetCFlagBaud(int baudrate)
{
    switch(baudrate)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}
