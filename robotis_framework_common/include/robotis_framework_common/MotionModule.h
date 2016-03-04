/*
 * MotionModule.h
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_MSGS_INCLUDE_MOTIONMODULE_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_MSGS_INCLUDE_MOTIONMODULE_H_


#include <map>
#include <string>

#include "robotis_device/Dynamixel.h"
#include "robotis_device/DynamixelState.h"

namespace ROBOTIS
{

enum CONTROL_MODE
{
    POSITION_CONTROL,
    VELOCITY_CONTROL,
    TORQUE_CONTROL
};

class MotionModule
{
public:
    bool            enable;
    std::string     module_name;
    CONTROL_MODE    control_mode;

    std::map<std::string, DynamixelState *> result;

    virtual ~MotionModule() { }

    virtual void    Initialize(const int control_cycle_msec) = 0;
    virtual void    Process(std::map<std::string, Dynamixel *> dxls) = 0;

    inline double powDI(double a, int b);
};

inline double MotionModule::powDI(double a, int b)
{
	return (b == 0 ? 1 : (b > 0 ? a * powDI(a, b - 1) : 1 / powDI(a, -b)));
}

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_MSGS_INCLUDE_MOTIONMODULE_H_ */
