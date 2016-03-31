/*
 * GroupSyncWrite.cpp
 *
 *  Created on: 2016. 1. 28.
 *      Author: zerom
 */

#include <algorithm>
#include "dynamixel_sdk/GroupSyncWrite.h"

using namespace ROBOTIS;

GroupSyncWrite::GroupSyncWrite(PortHandler *port, PacketHandler *ph, UINT16_T start_address, UINT16_T data_length)
    : port_(port),
      ph_(ph),
      is_param_changed_(false),
      param_(0),
      start_address_(start_address),
      data_length_(data_length)
{
    ClearParam();
}

void GroupSyncWrite::MakeParam()
{
    if(id_list_.size() == 0)
        return;

    if(param_ != 0)
        delete[] param_;
    param_ = 0;

    param_ = new UINT8_T[id_list_.size() * (1 + data_length_)]; // ID(1) + DATA(data_length)

    int _idx = 0;
    for(unsigned int _i = 0; _i < id_list_.size(); _i++)
    {
        UINT8_T _id = id_list_[_i];
        if(data_list_[_id] == 0)
            return;

        param_[_idx++] = _id;
        for(int _c = 0; _c < data_length_; _c++)
            param_[_idx++] = (data_list_[_id])[_c];
    }
}

bool GroupSyncWrite::AddParam(UINT8_T id, UINT8_T *data)
{
    if(std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
        return false;

    id_list_.push_back(id);
    data_list_[id] = new UINT8_T[data_length_];
    for(int _c = 0; _c < data_length_; _c++)
        data_list_[id][_c] = data[_c];

    is_param_changed_   = true;
    return true;
}

void GroupSyncWrite::RemoveParam(UINT8_T id)
{
    std::vector<UINT8_T>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
    if(it == id_list_.end())    // NOT exist
        return;

    id_list_.erase(it);
    delete[] data_list_[id];
    data_list_.erase(id);

    is_param_changed_   = true;
}

bool GroupSyncWrite::ChangeParam(UINT8_T id, UINT8_T *data)
{
    std::vector<UINT8_T>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
    if(it == id_list_.end())    // NOT exist
        return false;

    delete[] data_list_[id];
    data_list_[id] = new UINT8_T[data_length_];
    for(int _c = 0; _c < data_length_; _c++)
        data_list_[id][_c] = data[_c];

    is_param_changed_   = true;
    return true;
}

void GroupSyncWrite::ClearParam()
{
    if(id_list_.size() != 0)
    {
        for(unsigned int _i = 0; _i < id_list_.size(); _i++)
            delete[] data_list_[id_list_[_i]];
    }

    id_list_.clear();
    data_list_.clear();
    if(param_ != 0)
        delete[] param_;
    param_ = 0;
}

int GroupSyncWrite::TxPacket()
{
    if(id_list_.size() == 0)
        return COMM_NOT_AVAILABLE;

    if(is_param_changed_ == true)
        MakeParam();

    return ph_->SyncWriteTxOnly(port_, start_address_, data_length_, param_, id_list_.size() * (1 + data_length_));
}
