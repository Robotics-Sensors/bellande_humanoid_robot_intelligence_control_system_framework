/*
 * Sensor.h
 *
 *  Created on: 2015. 12. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_SENSOR_H_
#define ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_SENSOR_H_

#include <map>
#include <string>
#include <stdint.h>
#include "Device.h"
#include "SensorState.h"
#include "ControlTableItem.h"

namespace ROBOTIS
{

class Sensor : public Device
{
public:
    SensorState *sensor_state;

    Sensor(int id, std::string model_name, float protocol_version);
};

}


#endif /* ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_SENSOR_H_ */
