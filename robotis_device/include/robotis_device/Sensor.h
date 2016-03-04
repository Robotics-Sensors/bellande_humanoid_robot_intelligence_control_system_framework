/*
 * Sensor.h
 *
 *  Created on: 2015. 12. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_SENSOR_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_SENSOR_H_

#include <map>
#include <string>
#include <stdint.h>
#include "ControlTableItem.h"

namespace ROBOTIS
{

class Sensor
{
public:
    UINT8_T     id;
    std::string model_name;
    float       protocol_version;

    std::map<UINT16_T, ControlTableItem *> ctrl_table;

    Sensor(int id, std::string model_name, float protocol_version);
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_SENSOR_H_ */
