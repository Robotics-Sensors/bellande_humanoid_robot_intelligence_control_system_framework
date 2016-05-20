/*
 * MotionModule.h
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_MOTIONMODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_MOTIONMODULE_H_


#include <map>
#include <string>

#include "robotis_device/Robot.h"
#include "robotis_device/Dynamixel.h"
#include "robotis_framework_common/Singleton.h"

namespace ROBOTIS
{

enum CONTROL_MODE
{
    POSITION_CONTROL,
    VELOCITY_CONTROL,
    CURRENT_CONTROL
};

class MotionModule
{
public:
    bool            enable;
    std::string     module_name;
    CONTROL_MODE    control_mode;

    std::map<std::string, DynamixelState *> result;

    virtual ~MotionModule() { }

    virtual void    Initialize(const int control_cycle_msec, Robot *robot) = 0;
    virtual void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors) = 0;

    virtual void	Stop() = 0;
    virtual bool	IsRunning() = 0;
};


}


#endif /* ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_MOTIONMODULE_H_ */
