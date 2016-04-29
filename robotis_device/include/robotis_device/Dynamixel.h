/*
 * Dynamixel.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_


#include <map>
#include <vector>
#include <string>
#include "DynamixelState.h"
#include "ControlTableItem.h"

namespace ROBOTIS
{

class Dynamixel
{
public:
    UINT8_T     id;
    std::string model_name;
    float       protocol_version;

    std::map<std::string, ControlTableItem *> ctrl_table;   // string: item name

    std::string port_name;
    std::string ctrl_module_name;
    DynamixelState *dxl_state;

    double      current_ratio;
    double      velocity_ratio;

    INT32_T     value_of_0_radian_position;
    INT32_T     value_of_min_radian_position;
    INT32_T     value_of_max_radian_position;
    double      min_radian;
    double      max_radian;

    ControlTableItem   *torque_enable_item;
    ControlTableItem   *present_position_item;
    ControlTableItem   *present_velocity_item;
    ControlTableItem   *present_current_item;
    ControlTableItem   *goal_position_item;
    ControlTableItem   *goal_velocity_item;
    ControlTableItem   *goal_current_item;

    std::vector<ControlTableItem *> bulk_read_items;
    std::map<std::string, UINT16_T> indirect_address_table;

    Dynamixel(int id, std::string model_name, float protocol_version);

    double      ConvertValue2Radian(INT32_T value);
    INT32_T     ConvertRadian2Value(double radian);

    double      ConvertValue2Velocity(INT32_T value);
    INT32_T     ConvertVelocity2Value(double velocity);

    double      ConvertValue2Current(INT16_T value);
    INT16_T     ConvertCurrent2Value(double torque);
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_ */
