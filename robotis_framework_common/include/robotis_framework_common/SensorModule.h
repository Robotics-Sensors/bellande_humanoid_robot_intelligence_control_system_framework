/*
 * SensorModule.h
 *
 *  Created on: 2016. 3. 30.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_SENSORMODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_SENSORMODULE_H_


#include <map>
#include <string>

#include "robotis_device/Robot.h"
#include "robotis_device/Dynamixel.h"

namespace ROBOTIS
{

class SensorModule
{
public:
    std::string module_name;

    std::map<std::string, double> result;

    virtual ~SensorModule() { }

    virtual void    Initialize(const int control_cycle_msec, Robot *robot) = 0;
    virtual void    Process(std::map<std::string, Dynamixel *> dxls) = 0;
};

}


#endif /* ROBOTIS_FRAMEWORK_COMMON_INCLUDE_ROBOTIS_FRAMEWORK_COMMON_SENSORMODULE_H_ */
