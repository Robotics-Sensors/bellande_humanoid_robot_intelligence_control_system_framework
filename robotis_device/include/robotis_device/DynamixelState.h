/*
 * DynamixelState.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXELSTATE_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXELSTATE_H_

#include <stdint.h>
#include <robotis_framework_common/RobotisDef.h>

namespace ROBOTIS
{

class TimeStamp {
public:
    long sec;
    long nsec;
    TimeStamp(long sec, long nsec) : sec(sec), nsec(nsec) { }
};

class DynamixelState
{
public:
    TimeStamp   update_time_stamp;

    bool        torque_enable;
    UINT16_T    position_d_gain;
    UINT16_T    position_i_gain;
    UINT16_T    position_p_gain;
    UINT16_T    velocity_d_gain;
    UINT16_T    velocity_i_gain;
    UINT16_T    velocity_p_gain;
    double      goal_position;
    double      goal_velocity;
    double      goal_torque;
    double      present_position;
    double      present_velocity;
    double      present_load;
    bool        is_moving;

    UINT16_T    ext_port_data[4];

    double      position_offset;

    DynamixelState()
        : update_time_stamp(0, 0),
          torque_enable(false),
          position_d_gain(0),
          position_i_gain(0),
          position_p_gain(0),
          velocity_d_gain(0),
          velocity_i_gain(0),
          velocity_p_gain(0),
          goal_position(0.0),
          goal_velocity(0.0),
          goal_torque(0.0),
          present_position(0.0),
          present_velocity(0.0),
          present_load(0.0),
          is_moving(false),
          position_offset(0.0)
    {
        ext_port_data[0] = 2048;
        ext_port_data[1] = 2048;
        ext_port_data[2] = 2048;
        ext_port_data[3] = 2048;
    }
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_DEVICE_DYNAMIXELSTATE_H_ */
