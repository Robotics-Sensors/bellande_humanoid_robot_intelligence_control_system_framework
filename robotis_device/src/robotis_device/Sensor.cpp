/*
 * Sensor.cpp
 *
 *  Created on: 2016. 5. 11.
 *      Author: zerom
 */

#include "robotis_device/Sensor.h"

using namespace ROBOTIS;

Sensor::Sensor(int id, std::string model_name, float protocol_version)
{
    this->id = id;
    this->model_name = model_name;
    this->port_name = "";
    this->protocol_version = protocol_version;
    ctrl_table.clear();

    sensor_state = new SensorState();

    bulk_read_items.clear();
}
