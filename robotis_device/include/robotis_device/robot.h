/*
 * Robot.h
 *
 *  Created on: 2015. 12. 11.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_ROBOT_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_ROBOT_H_


#include <vector>
#include "Sensor.h"
#include "Dynamixel.h"
#include "dynamixel_sdk/PortHandler.h"

namespace ROBOTIS
{

class Robot
{
public:
    std::map<std::string, PortHandler *>    ports;      // string: port name
    std::map<std::string, std::string>      port_default_device; // port name, default device name

    std::map<std::string, Dynamixel *>      dxls;       // string: joint name
    std::map<std::string, Sensor *>         sensors;    // string: sensor name

    Robot(std::string robot_file_path, std::string dev_desc_dir_path);

    Sensor     *getSensor(std::string path, int id, std::string port, float protocol_version);
    Dynamixel  *getDynamixel(std::string path, int id, std::string port, float protocol_version);
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_DEVICE_INCLUDE_DEVICE_ROBOT_H_ */
