/*
 * Dynamixel.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_


#include <map>
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

    INT32_T     zero_radian_position_value;
    INT32_T     min_radian_position_value;
    INT32_T     max_radian_position_value;
    double      min_radian;
    double      max_radian;

    UINT16_T    torque_enable_address;
    UINT16_T    position_d_gain_address;
    UINT16_T    position_i_gain_address;
    UINT16_T    position_p_gain_address;
    UINT16_T    goal_position_address;
    UINT16_T    goal_velocity_address;
    UINT16_T    goal_torque_address;
    UINT16_T    present_position_address;
    UINT16_T    present_velocity_address;
    UINT16_T    present_load_address;
    UINT16_T    is_moving_address;

    Dynamixel(int id, std::string model_name, float protocol_version);

    double      ConvertValue2Radian(int32_t value)  { return value * max_radian / max_radian_position_value; }
    INT32_T     ConvertRadian2Value(double radian)  { return radian * max_radian_position_value / max_radian; }
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_ */
