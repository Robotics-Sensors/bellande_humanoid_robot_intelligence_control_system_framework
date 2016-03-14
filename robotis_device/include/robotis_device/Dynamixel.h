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

    INT32_T     value_of_0_radian_position;
    INT32_T     value_of_min_radian_position;
    INT32_T     value_of_max_radian_position;
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

    double      ConvertValue2Radian(INT32_T value);
    INT32_T     ConvertRadian2Value(double radian);
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXEL_H_ */
