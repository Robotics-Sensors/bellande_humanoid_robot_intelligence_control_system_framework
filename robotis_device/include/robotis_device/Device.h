/*
 * Device.h
 *
 *  Created on: 2016. 5. 12.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_DEVICE_H_
#define ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_DEVICE_H_


#include <map>
#include <string>
#include <vector>
#include "ControlTableItem.h"

namespace ROBOTIS
{

class Device
{
public:
    UINT8_T     id;
    float       protocol_version;
    std::string model_name;
    std::string port_name;

    std::map<std::string, ControlTableItem *>  ctrl_table;
    std::vector<ControlTableItem *>         bulk_read_items;

    virtual ~Device() { }
};

}


#endif /* ROBOTIS_DEVICE_INCLUDE_ROBOTIS_DEVICE_DEVICE_H_ */
