/*
 * ControlTableItem.h
 *
 *  Created on: 2015. 12. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_CONTROLTABLEITEM_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_CONTROLTABLEITEM_H_


#include <robotis_framework_common/RobotisDef.h>

namespace ROBOTIS
{

enum ACCESS_TYPE {
    READ,
    READ_WRITE
};

enum MEMORY_TYPE {
    EEPROM,
    RAM
};

class ControlTableItem
{
public:
    std::string item_name;
    UINT16_T    address;
    ACCESS_TYPE access_type;
    MEMORY_TYPE memory_type;
    UINT8_T     data_length;
    INT32_T     data_min_value;
    INT32_T     data_max_value;
    bool        is_signed;

    ControlTableItem()
        : item_name(""),
          address(0),
          access_type(READ),
          memory_type(RAM),
          data_length(0),
          data_min_value(0),
          data_max_value(0),
          is_signed(false)
    { }
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_CONTROLTABLEITEM_H_ */
