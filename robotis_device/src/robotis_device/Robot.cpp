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
                if(tokens.size() != 2)
                    continue;

                std::cout << tokens[0] << " added. (baudrate: " << tokens[1] << ")" << std::endl;

                ports[tokens[0]] = (PortHandler*)PortHandler::GetPortHandler(tokens[0].c_str());
                ports[tokens[0]]->SetBaudRate(std::atoi(tokens[1].c_str()));
            }
            else if(session == "device info")
            {
                std::vector<std::string> tokens = split(input_str, '|');
                if(tokens.size() != 6)
                    continue;

                if(tokens[0] == "dynamixel")
                {
                    std::string _file_path  = dev_desc_dir_path + tokens[0] + "/" + tokens[3] + ".device";
                    int         _id         = std::atoi(tokens[2].c_str());
                    std::string _port       = tokens[1];
                    float       _protocol   = std::atof(tokens[4].c_str());
                    std::string _dev_name   = tokens[5];

                    dxls[_dev_name] = getDynamixel(_file_path, _id, _port, _protocol);
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

Dynamixel *Robot::getDynamixel(std::string path, int id, std::string port, float protocol_version)
{
    Dynamixel *dxl;
    // load file model_name.device
    std::ifstream file(path.c_str());
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
            if(input_str == "")
                continue;

            // find session
            if(!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size()-1, 1, "]"))
            {
                input_str = input_str.substr(1, input_str.size()-2);
                std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
                session = trim(input_str);
                continue;
            }

            if(session == "device info")
            {
                std::vector<std::string> tokens = split(input_str, '=');
                if(tokens.size() != 2)
                    continue;

                if(tokens[0] == "model_name")
                    dxl = new Dynamixel(id, tokens[1], protocol_version);
            }
            else if(session == "type info")
            {
                std::vector<std::string> tokens = split(input_str, '=');
                if(tokens.size() != 2)
                    continue;

                if(tokens[0] == "0_radian_position_value")
                    dxl->zero_radian_position_value = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "min_radian_position_value")
                    dxl->min_radian_position_value = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "max_radian_position_value")
                    dxl->max_radian_position_value = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "min_radian")
                    dxl->min_radian = std::atof(tokens[1].c_str());
                else if(tokens[0] == "max_radian")
                    dxl->max_radian = std::atof(tokens[1].c_str());
                else if(tokens[0] == "torque_enable_address")
                    dxl->torque_enable_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "position_d_gain_address")
                    dxl->position_d_gain_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "position_i_gain_address")
                    dxl->position_i_gain_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "position_p_gain_address")
                    dxl->position_p_gain_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "goal_position_address")
                    dxl->goal_position_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "goal_velocity_address")
                    dxl->goal_velocity_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "goal_torque_address")
                    dxl->goal_torque_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "present_position_address")
                    dxl->present_position_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "present_velocity_address")
                    dxl->present_velocity_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "present_load_address")
                    dxl->present_load_address = std::atoi(tokens[1].c_str());
                else if(tokens[0] == "is_moving_address")
                    dxl->is_moving_address = std::atoi(tokens[1].c_str());
            }
            else if(session == "control table")
            {
                std::vector<std::string> tokens = split(input_str, '|');
                if(tokens.size() != 8)
                    continue;

                ControlTableItem *item = new ControlTableItem();
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
                dxl->ctrl_table[tokens[1]] = item;
            }
        }
        dxl->port_name = port;

        fprintf(stderr, "(%s) [ID:%3d] %14s added. \n", port.c_str(), dxl->id, dxl->model_name.c_str());
        //std::cout << "[ID:" << (int)(dxl->id) << "] " << dxl->model_name << " added. (" << port << ")" << std::endl;
        file.close();
    }
    else
        std::cout << "Unable to open file : " + path << std::endl;

    return dxl;
}

