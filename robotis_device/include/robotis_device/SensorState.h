/*
 * SensorState.h
 *
 *  Created on: 2016. 5. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_SENSORSTATE_H_
#define ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_SENSORSTATE_H_


#include "robotis_device/TimeStamp.h"
#include "robotis_framework_common/RobotisDef.h"

namespace ROBOTIS
{

class SensorState
{
public:
    TimeStamp   update_time_stamp;

    std::map<std::string, UINT32_T> bulk_read_table;

    SensorState()
        : update_time_stamp(0, 0)
    {
        bulk_read_table.clear();
    }
};

}


#endif /* ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_SENSORSTATE_H_ */
