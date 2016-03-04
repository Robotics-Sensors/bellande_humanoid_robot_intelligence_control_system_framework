/*
 * Dynamixel.cpp
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#include "../include/robotis_device/Dynamixel.h"

using namespace ROBOTIS;

Dynamixel::Dynamixel(int id, std::string model_name, float protocol_version)
    : id(id),
      model_name(model_name),
      port_name(""),
      ctrl_module_name("none"),
      protocol_version(protocol_version),
      zero_radian_position_value(0),
      min_radian_position_value(0),
      max_radian_position_value(0),
      min_radian(-3.14),
      max_radian(3.14),
      torque_enable_address(0),
      position_d_gain_address(0),
      position_i_gain_address(0),
      position_p_gain_address(0),
      goal_position_address(0),
      goal_velocity_address(0),
      goal_torque_address(0),
      present_position_address(0),
      present_velocity_address(0),
      present_load_address(0),
      is_moving_address(0)
{
    ctrl_table.clear();
    dxl_state = new DynamixelState();
}
