/*
 * GroupBulkRead.cpp
 *
 *  Created on: 2016. 1. 28.
 *      Author: zerom
 */

#include <stdio.h>
#include <algorithm>
#include "dynamixel_sdk/GroupBulkRead.h"

using namespace ROBOTIS;

GroupBulkRead::GroupBulkRead(PortHandler *port, PacketHandler *ph)
    : port_(port),
      ph_(ph),
      param_(0)
{
    ClearParam();
}

void GroupBulkRead::MakeParam()
{
    if(id_list_.size() == 0)
        return;

    if(param_ != 0)
        delete[] param_;
    param_ = 0;

    if(ph_->GetProtocolVersion() == 1.0)
        param_ = new UINT8_T[id_list_.size() * 3];  // ID(1) + ADDR(1) + LENGTH(1)
    else    // 2.0
        param_ = new UINT8_T[id_list_.size() * 5];  // ID(1) + ADDR(2) + LENGTH(2)

    int _idx = 0;
    for(int _i = 0; _i < id_list_.size(); _i++)
    {
        UINT8_T _id = id_list_[_i];
        if(ph_->GetProtocolVersion() == 1.0)
        {
            param_[_idx++] = (UINT8_T)length_list_[_id];    // LEN
            param_[_idx++] = _id;                           // ID
            param_[_idx++] = (UINT8_T)address_list_[_id];   // ADDR
        }
        else    // 2.0
        {
            param_[_idx++] = _id;                               // ID
            param_[_idx++] = DXL_LOBYTE(address_list_[_id]);    // ADDR_L
            param_[_idx++] = DXL_HIBYTE(address_list_[_id]);    // ADDR_H
            param_[_idx++] = DXL_LOBYTE(length_list_[_id]);     // LEN_L
            param_[_idx++] = DXL_HIBYTE(length_list_[_id]);     // LEN_H
        }
    }
}

bool GroupBulkRead::AddParam(UINT8_T id, UINT16_T start_address, UINT16_T data_length)
{
    if(std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
        return false;

    id_list_.push_back(id);
    length_list_[id]    = data_length;
    address_list_[id]   = start_address;
    data_list_[id]      = new UINT8_T[data_length];

    MakeParam();
    return true;
}

void GroupBulkRead::RemoveParam(UINT8_T id)
{
    std::vector<UINT8_T>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
    if(it == id_list_.end())    // NOT exist
        return;

    id_list_.erase(it);
    address_list_.erase(id);
    length_list_.erase(id);
    delete[] data_list_[id];
    data_list_.erase(id);

    MakeParam();
}

void GroupBulkRead::ClearParam()
{
    if(id_list_.size() != 0)
    {
        for(int _i = 0; _i < id_list_.size(); _i++)
            delete[] data_list_[id_list_[_i]];
    }

    id_list_.clear();
    address_list_.clear();
    length_list_.clear();
    data_list_.clear();
    if(param_ != 0)
        delete[] param_;
    param_ = 0;
}

int GroupBulkRead::TxPacket()
{
    if(id_list_.size() == 0)
        return COMM_NOT_AVAILABLE;

    if(ph_->GetProtocolVersion() == 1.0)
        return ph_->BulkReadTx(port_, param_, id_list_.size() * 3);
    else    // 2.0
        return ph_->BulkReadTx(port_, param_, id_list_.size() * 5);
}

int GroupBulkRead::RxPacket()
{
    int _cnt            = id_list_.size();
    int _result         = COMM_RX_FAIL;

    if(_cnt == 0)
        return COMM_NOT_AVAILABLE;

    for(int _i = 0; _i < _cnt; _i++)
    {
        UINT8_T _id = id_list_[_i];

        _result = ph_->ReadRx(port_, length_list_[_id], data_list_[_id]);
        if(_result != COMM_SUCCESS)
        {
            fprintf(stderr, "[GroupBulkRead::RxPacket] ID %d result : %d !!!!!!!!!!\n", _id, _result);
            return _result;
        }
    }

    return _result;
}

int GroupBulkRead::TxRxPacket()
{
    int _result         = COMM_TX_FAIL;

    _result = TxPacket();
    if(_result != COMM_SUCCESS)
        return _result;

    return RxPacket();
}

bool GroupBulkRead::GetData(UINT8_T id, UINT16_T address, UINT8_T *data)
{
    UINT16_T _start_addr, _data_length;

    if(data_list_.find(id) == data_list_.end())
        return false;

    _start_addr = address_list_[id];
    _data_length = length_list_[id];

    if(address < _start_addr || _start_addr + _data_length - 1 < address)
        return false;

    *data = data_list_[id][address - _start_addr];

    return true;
}

bool GroupBulkRead::GetData(UINT8_T id, UINT16_T address, UINT16_T *data)
{
    UINT16_T _start_addr, _data_length;

    if(data_list_.find(id) == data_list_.end())
        return false;

    _start_addr = address_list_[id];
    _data_length = length_list_[id];

    if(address < _start_addr || _start_addr + _data_length - 2 < address)
        return false;

    *data = DXL_MAKEWORD(data_list_[id][address - _start_addr], data_list_[id][address - _start_addr + 1]);

    return true;
}

bool GroupBulkRead::GetData(UINT8_T id, UINT16_T address, UINT32_T *data)
{
    UINT16_T _start_addr, _data_length;

    if(data_list_.find(id) == data_list_.end())
        return false;

    _start_addr = address_list_[id];
    _data_length = length_list_[id];

    if(address < _start_addr || _start_addr + _data_length - 4 < address)
        return false;

    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][address - _start_addr + 0], data_list_[id][address - _start_addr + 1]),
                          DXL_MAKEWORD(data_list_[id][address - _start_addr + 2], data_list_[id][address - _start_addr + 3]));

    return true;
}

