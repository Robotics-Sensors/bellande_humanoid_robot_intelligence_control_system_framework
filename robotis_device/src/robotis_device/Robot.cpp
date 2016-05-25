/*
 * Robot.cpp
 *
 *  Created on: 2015. 12. 11.
 *      Author: zerom
 */

#include <fstream>
#include <iostream>
#include <algorithm>
#include "../include/robotis_device/Robot.h"

using namespace ROBOTIS;

static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

static inline std::vector<std::string> split(const std::string &text, char sep) {
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    while((end = text.find(sep, start)) != (std::string::npos)) {
        tokens.push_back(text.substr(start, end - start));
        trim(tokens.back());
        start = end + 1;
    }
    tokens.push_back(text.substr(start));
    trim(tokens.back());
    return tokens;
}

Robot::Robot(std::string robot_file_path, std::string dev_desc_dir_path)
{
    if(dev_desc_dir_path.compare(dev_desc_dir_path.size()-1, 1, "/") != 0)
        dev_desc_dir_path += "/";

    std::ifstream file(robot_file_path.c_str());
    if(file.is_open())
    {
        std::string session = "";
        std::string input_str;
        while(!file.eof())
        {
            std::getline(file, input_str);

            // remove comment ( # )
            std::size_t pos = input_str.find("#");
            if(pos != std::string::npos)
                input_str = input_str.substr(0, pos);

            // trim
            input_str = trim(input_str);

            // find session
            if(!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size()-1, 1, "]"))
            {
                input_str = input_str.substr(1, input_str.size()-2);
                std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
                session = trim(input_str);
                continue;
            }

            if(session == "port info")
            {
                std::vector<std::string> tokens = split(input_str, '|');
                if(tokens.size() != 3)
                    continue;

                std::cout << tokens[0] << " added. (baudrate: " << tokens[1] << ")" << std::endl;

                ports[tokens[0]] = (PortHandler*)PortHandler::GetPortHandler(tokens[0].c_str());
                ports[tokens[0]]->SetBaudRate(std::atoi(tokens[1].c_str()));
                port_default_device[tokens[0]] = tokens[2];
            }
            else if(session == "device info")
            {
                std::vector<std::string> tokens = split(input_str, '|');
                if(tokens.size() != 7)
                    continue;

                if(tokens[0] == "dynamixel")
                {
                    std::string _file_path  = dev_desc_dir_path + tokens[0] + "/" + tokens[3] + ".device";
                    int         _id         = std::atoi(tokens[2].c_str());
                    std::string _port       = tokens[1];
                    float       _protocol   = std::atof(tokens[4].c_str());
                    std::string _dev_name   = tokens[5];

                    dxls[_dev_name] = getDynamixel(_file_path, _id, _port, _protocol);

                    Dynamixel *_dxl = dxls[_dev_name];
                    std::vector<std::string> sub_tokens = split(tokens[6], ',');
                    if(sub_tokens.size() > 0)
                    {
                        std::map<std::string, ControlTableItem *>::iterator _indirect_it = _dxl->ctrl_table.find(INDIRECT_ADDRESS_1);
                        if(_indirect_it != _dxl->ctrl_table.end())    // INDIRECT_ADDRESS_1 exist
                        {
                            UINT16_T _indirect_data_addr = _dxl->ctrl_table[INDIRECT_DATA_1]->address;
                            for(int _i = 0; _i < sub_tokens.size(); _i++)
                            {
                                if(_dxl->bulk_read_items[_i] == NULL)
                                    continue;

                                _dxl->bulk_read_items.push_back(new ControlTableItem());
                                ControlTableItem *_dest_item = _dxl->bulk_read_items[_i];
                                ControlTableItem *_src_item = _dxl->ctrl_table[sub_tokens[_i]];

                                _dest_item->item_name   = _src_item->item_name;
                                _dest_item->address     = _indirect_data_addr;
                                _dest_item->access_type = _src_item->access_type;
                                _dest_item->memory_type = _src_item->memory_type;
                                _dest_item->data_length = _src_item->data_length;
                                _dest_item->data_min_value = _src_item->data_min_value;
                                _dest_item->data_max_value = _src_item->data_max_value;
                                _dest_item->is_signed   = _src_item->is_signed;

                                _indirect_data_addr += _dest_item->data_length;
                            }
                        }
                        else    // INDIRECT_ADDRESS_1 not exist
                        {
                            for(int _i = 0; _i < sub_tokens.size(); _i++)
                            {
                                if(_dxl->ctrl_table[sub_tokens[_i]] != NULL)
                                    _dxl->bulk_read_items.push_back(_dxl->ctrl_table[sub_tokens[_i]]);
                            }
                        }
                    }
                }
                else if(tokens[0] == "sensor")
                {
                    std::string _file_path  = dev_desc_dir_path + tokens[0] + "/" + tokens[3] + ".device";
                    int         _id         = std::atoi(tokens[2].c_str());
                    std::string _port       = tokens[1];
                    float       _protocol   = std::atof(tokens[4].c_str());
                    std::string _dev_name   = tokens[5];

                    sensors[_dev_name] = getSensor(_file_path, _id, _port, _protocol);

                    Sensor *_sensor = sensors[_dev_name];
                    std::vector<std::string> sub_tokens = split(tokens[6], ',');
                    if(sub_tokens.size() > 0)
                    {
                        std::map<std::string, ControlTableItem *>::iterator _indirect_it = _sensor->ctrl_table.find(INDIRECT_ADDRESS_1);
                        if(_indirect_it != _sensor->ctrl_table.end())    // INDIRECT_ADDRESS_1 exist
                        {
                            UINT16_T _indirect_data_addr = _sensor->ctrl_table[INDIRECT_DATA_1]->address;
                            for(int _i = 0; _i < sub_tokens.size(); _i++)
                            {
                                _sensor->bulk_read_items.push_back(new ControlTableItem());
                                ControlTableItem *_dest_item = _sensor->bulk_read_items[_i];
                                ControlTableItem *_src_item = _sensor->ctrl_table[sub_tokens[_i]];

                                _dest_item->item_name   = _src_item->item_name;
                                _dest_item->address     = _indirect_data_addr;
                                _dest_item->access_type = _src_item->access_type;
                                _dest_item->memory_type = _src_item->memory_type;
                                _dest_item->data_length = _src_item->data_length;
                                _dest_item->data_min_value = _src_item->data_min_value;
                                _dest_item->data_max_value = _src_item->data_max_value;
                                _dest_item->is_signed   = _src_item->is_signed;

                                _indirect_data_addr += _dest_item->data_length;
                            }
                        }
                        else    // INDIRECT_ADDRESS_1 exist
                        {
                            for(int _i = 0; _i < sub_tokens.size(); _i++)
                                _sensor->bulk_read_items.push_back(_sensor->ctrl_table[sub_tokens[_i]]);
                        }
                    }
                }
            }
        }
        file.close();
    }
    else
    {
        std::cout << "Unable to open file : " + robot_file_path << std::endl;
    }
}

Sensor *Robot::getSensor(std::string path, int id, std::string port, float protocol_version)
{
    Sensor *_sensor;

    // load file model_name.device
    std::ifstream file(path.c_str());
    if(file.is_open())
    {
        std::string _session = "";
        std::string _input_str;

        while(!file.eof())
        {
            std::getline(file, _input_str);

            // remove comment ( # )
            std::size_t pos = _input_str.find("#");
            if(pos != std::string::npos)
                _input_str = _input_str.substr(0, pos);

            // trim
            _input_str = trim(_input_str);
            if(_input_str == "")
                continue;

            // find _session
            if(!_input_str.compare(0, 1, "[") && !_input_str.compare(_input_str.size()-1, 1, "]"))
            {
                _input_str = _input_str.substr(1, _input_str.size()-2);
                std::transform(_input_str.begin(), _input_str.end(), _input_str.begin(), ::tolower);
                _session = trim(_input_str);
                continue;
            }

            if(_session == "device info")
            {
                std::vector<std::string> tokens = split(_input_str, '=');
                if(tokens.size() != 2)
                    continue;

                if(tokens[0] == "model_name")
                    _sensor = new Sensor(id, tokens[1], protocol_version);
            }
            else if(_session == "control table")
            {
                std::vector<std::string> tokens = split(_input_str, '|');
                if(tokens.size() != 8)
                    continue;

                ControlTableItem *item = new ControlTableItem();
                item->item_name = tokens[1];
                item->address = std::atoi(tokens[0].c_str());
                item->data_length = std::atoi(tokens[2].c_str());
                if(tokens[3] == "R")
                    item->access_type = READ;
                else if(tokens[3] == "RW")
                    item->access_type = READ_WRITE;
                if(tokens[4] == "EEPROM")
                    item->memory_type = EEPROM;
                else if(tokens[4] == "RAM")
                    item->memory_type = RAM;
                item->data_min_value = std::atol(tokens[5].c_str());
                item->data_max_value = std::atol(tokens[6].c_str());
                if(tokens[7] == "Y")
                    item->is_signed = true;
                else if(tokens[7] == "N")
                    item->is_signed = false;
                _sensor->ctrl_table[tokens[1]] = item;
            }
        }
        _sensor->port_name = port;

        fprintf(stderr, "(%s) [ID:%3d] %14s added. \n", port.c_str(), _sensor->id, _sensor->model_name.c_str());
        //std::cout << "[ID:" << (int)(_sensor->id) << "] " << _sensor->model_name << " added. (" << port << ")" << std::endl;
        file.close();
    }
    else
        std::cout << "Unable to open file : " + path << std::endl;

    return _sensor;
}

Dynamixel *Robot::getDynamixel(std::string path, int id, std::string port, float protocol_version)
{
    Dynamixel *_dxl;

    // load file model_name.device
    std::ifstream file(path.c_str());
    if(file.is_open())
    {
        std::string _session = "";
        std::string _input_str;

        std::string _torque_enable_item_name    = "";
        std::string _present_position_item_name = "";
        std::string _present_velocity_item_name = "";
        std::string _present_current_item_name  = "";
        std::string _goal_position_item_name    = "";
        std::string _goal_velocity_item_name    = "";
        std::string _goal_current_item_name     = "";

        while(!file.eof())
        {
            std::getline(file, _input_str);

            // remove comment ( # )
            std::size_t pos = _input_str.find("#");
            if(pos != std::string::npos)
                _input_str = _input_str.substr(0, pos);

            // trim
            _input_str = trim(_input_str);
            if(_input_str == "")
                continue;

            // find _session
            if(!_input_str.compare(0, 1, "[") && !_input_str.compare(_input_str.size()-1, 1, "]"))
            {
                _input_str = _input_str.substr(1, _input_str.size()-2);
                std::transform(_input_str.begin(), _input_str.end(), _input_str.begin(), ::tolower);
                _session = trim(_input_str);
                continue;
            }

            if(_session == "device info")
            {
                std::vector<std::string> tokens = split(_input_str, '=');
                if(tokens.size() != 2)
                    continue;

                if(tokens[0] == "model_name")
                    _dxl = new Dynamixel(id, tokens[1], protocol_version);
            }
            else if(_session == "type info")
            {
                std::vector<std::string> tokens = split(_input_str, '=');
                if(tokens.size() != 2)
                    continue;

                if(tokens[0] == "current_ratio")
                	_dxl->current_ratio = std::atof(tokens[1].c_str());
                else if(tokens[0] == "velocity_ratio")
                    _dxl->velocity_ratio = std::atof(tokens[1].c_str());

                else if(tokens[0] == "value_of_0_radian_position")
                    _dxl->value_of_0_radian_position = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "value_of_min_radian_position")
                    _dxl->value_of_min_radian_position = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "value_of_max_radian_position")
                    _dxl->value_of_max_radian_position = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "min_radian")
                    _dxl->min_radian = std::atof(tokens[1].c_str());
                else if(tokens[0] == "max_radian")
                    _dxl->max_radian = std::atof(tokens[1].c_str());

                else if(tokens[0] == "torque_enable_item_name")
                    _torque_enable_item_name = tokens[1];
                else if(tokens[0] == "present_position_item_name")
                    _present_position_item_name = tokens[1];
                else if(tokens[0] == "present_velocity_item_name")
                    _present_velocity_item_name = tokens[1];
                else if(tokens[0] == "present_current_item_name")
                    _present_current_item_name = tokens[1];
                else if(tokens[0] == "goal_position_item_name")
                    _goal_position_item_name = tokens[1];
                else if(tokens[0] == "goal_velocity_item_name")
                    _goal_velocity_item_name = tokens[1];
                else if(tokens[0] == "goal_current_item_name")
                    _goal_current_item_name = tokens[1];
            }
            else if(_session == "control table")
            {
                std::vector<std::string> tokens = split(_input_str, '|');
                if(tokens.size() != 8)
                    continue;

                ControlTableItem *item = new ControlTableItem();
                item->item_name = tokens[1];
                item->address = std::atoi(tokens[0].c_str());
                item->data_length = std::atoi(tokens[2].c_str());
                if(tokens[3] == "R")
                    item->access_type = READ;
                else if(tokens[3] == "RW")
                    item->access_type = READ_WRITE;
                if(tokens[4] == "EEPROM")
                    item->memory_type = EEPROM;
                else if(tokens[4] == "RAM")
                    item->memory_type = RAM;
                item->data_min_value = std::atol(tokens[5].c_str());
                item->data_max_value = std::atol(tokens[6].c_str());
                if(tokens[7] == "Y")
                    item->is_signed = true;
                else if(tokens[7] == "N")
                    item->is_signed = false;
                _dxl->ctrl_table[tokens[1]] = item;
            }
        }
        _dxl->port_name = port;

        if(_dxl->ctrl_table[_torque_enable_item_name] != NULL)
            _dxl->torque_enable_item = _dxl->ctrl_table[_torque_enable_item_name];
        if(_dxl->ctrl_table[_present_position_item_name] != NULL)
            _dxl->present_position_item = _dxl->ctrl_table[_present_position_item_name];
        if(_dxl->ctrl_table[_present_velocity_item_name] != NULL)
            _dxl->present_velocity_item = _dxl->ctrl_table[_present_velocity_item_name];
        if(_dxl->ctrl_table[_present_current_item_name] != NULL)
            _dxl->present_current_item = _dxl->ctrl_table[_present_current_item_name];
        if(_dxl->ctrl_table[_goal_position_item_name] != NULL)
            _dxl->goal_position_item = _dxl->ctrl_table[_goal_position_item_name];
        if(_dxl->ctrl_table[_goal_velocity_item_name] != NULL)
            _dxl->goal_velocity_item = _dxl->ctrl_table[_goal_velocity_item_name];
        if(_dxl->ctrl_table[_goal_current_item_name] != NULL)
            _dxl->goal_current_item = _dxl->ctrl_table[_goal_current_item_name];

        fprintf(stderr, "(%s) [ID:%3d] %14s added. \n", port.c_str(), _dxl->id, _dxl->model_name.c_str());
        //std::cout << "[ID:" << (int)(_dxl->id) << "] " << _dxl->model_name << " added. (" << port << ")" << std::endl;
        file.close();
    }
    else
        std::cout << "Unable to open file : " + path << std::endl;

    return _dxl;
}

