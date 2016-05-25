/*
 * Dynamixel.cpp
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#include "robotis_device/Dynamixel.h"

using namespace ROBOTIS;

Dynamixel::Dynamixel(int id, std::string model_name, float protocol_version)
    : ctrl_module_name("none"),
      current_ratio(1.0),
      velocity_ratio(1.0),
      value_of_0_radian_position(0),
      value_of_min_radian_position(0),
      value_of_max_radian_position(0),
      min_radian(-3.14159265),
      max_radian(3.14159265),
      torque_enable_item(0),
      present_position_item(0),
      present_velocity_item(0),
      present_current_item(0),
      goal_position_item(0),
      goal_velocity_item(0),
      goal_current_item(0),
      position_p_gain_item(0)
{
    this->id = id;
    this->model_name = model_name;
    this->port_name = "";
    this->protocol_version = protocol_version;

    ctrl_table.clear();
    dxl_state = new DynamixelState();

    bulk_read_items.clear();
}

double Dynamixel::ConvertValue2Radian(INT32_T value)
{
    double _radian = 0.0;
    if(value > value_of_0_radian_position)
    {
        if(max_radian <= 0)
            return max_radian;
        _radian = (double)(value - value_of_0_radian_position) * max_radian / (double)(value_of_max_radian_position - value_of_0_radian_position);
    }
    else if(value < value_of_0_radian_position)
    {
        if(min_radian >= 0)
            return min_radian;
        _radian = (double)(value - value_of_0_radian_position) * min_radian / (double)(value_of_min_radian_position - value_of_0_radian_position);
    }

    if(_radian > max_radian)
        return max_radian;
    else if(_radian < min_radian)
        return min_radian;

    return _radian;
}

INT32_T Dynamixel::ConvertRadian2Value(double radian)
{
    //return radian * value_of_max_radian_position / max_radian;

    INT32_T _value = 0;
    if(radian > 0)
    {
        if(value_of_max_radian_position <= value_of_0_radian_position)
            return value_of_max_radian_position;
        _value = (radian * (value_of_max_radian_position - value_of_0_radian_position) / max_radian) + value_of_0_radian_position;
    }
    else if(radian < 0)
    {
        if(value_of_min_radian_position >= value_of_0_radian_position)
            return value_of_min_radian_position;
        _value = (radian * (value_of_min_radian_position - value_of_0_radian_position) / min_radian) + value_of_0_radian_position;
    }
    else
        _value = value_of_0_radian_position;

    if(_value > value_of_max_radian_position)
        return value_of_max_radian_position;
    else if(_value < value_of_min_radian_position)
        return value_of_min_radian_position;

    return _value;
}

double Dynamixel::ConvertValue2Velocity(INT32_T value)
{
    return (double)value * velocity_ratio;
}

INT32_T Dynamixel::ConvertVelocity2Value(double velocity)
{
    return (INT32_T)(velocity / velocity_ratio);;
}

double Dynamixel::ConvertValue2Current(INT16_T value)
{
	return (double)value * current_ratio;
}

INT16_T Dynamixel::ConvertCurrent2Value(double torque)
{
	return (INT16_T)(torque / current_ratio);
}
