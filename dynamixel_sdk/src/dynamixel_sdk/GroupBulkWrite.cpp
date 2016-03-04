/*
 * GroupBulkWrite.cpp
 *
 *  Created on: 2016. 2. 2.
 *      Author: zerom
 */

#include <algorithm>
#include "dynamixel_sdk/GroupBulkWrite.h"

using namespace ROBOTIS;

GroupBulkWrite::GroupBulkWrite(PortHandler *port, PacketHandler *ph)
    : port_(port),
      ph_(ph),
      param_(0),
      param_length_(0)
{
    ClearParam();
}

void GroupBulkWrite::MakeParam()
{
    if(ph_->GetProtocolVersion() == 1.0 || id_list_.size() == 0)
        return;

    if(param_ != 0)
        delete[] param_;
    param_ = 0;

    param_length_ = 0;
    for(int _i = 0; _i < id_list_.size(); _i++)
        param_length_ += 1 + 2 + 2 + length_list_[id_list_[_i]];

    param_ = new UINT8_T[param_length_];

    int _idx = 0;
    for(int _i = 0; _i < id_list_.size(); _i++)
    {
        UINT8_T _id = id_list_[_i];
        if(data_list_[_id] == 0)
            return;

        param_[_idx++] = _id;
        param_[_idx++] = DXL_LOBYTE(address_list_[_id]);
        param_[_idx++] = DXL_HIBYTE(address_list_[_id]);
        param_[_idx++] = DXL_LOBYTE(length_list_[_id]);
        param_[_idx++] = DXL_HIBYTE(length_list_[_id]);
        for(int _c = 0; _c < length_list_[_id]; _c++)
            param_[_idx++] = (data_list_[_id])[_c];
    }
}

bool GroupBulkWrite::AddParam(UINT8_T id, UINT16_T start_address, UINT16_T data_length, UINT8_T *data)
{
    if(ph_->GetProtocolVersion() == 1.0)
        return false;

    if(std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
        return false;

    id_list_.push_back(id);
    address_list_[id]   = start_address;
    length_list_[id]    = data_length;
    data_list_[id] = new UINT8_T[data_length];
    for(int _c = 0; _c < data_length; _c++)
        data_list_[id][_c] = data[_c];

    MakeParam();
    return true;
}
void GroupBulkWrite::RemoveParam(UINT8_T id)
{
    if(ph_->GetProtocolVersion() == 1.0)
        return;

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
bool GroupBulkWrite::ChangeParam(UINT8_T id, UINT16_T start_address, UINT16_T data_length, UINT8_T *data)
{
    if(ph_->GetProtocolVersion() == 1.0)
            return false;

    std::vector<UINT8_T>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
    if(it == id_list_.end())    // NOT exist
        return false;

    address_list_[id] = start_address;
    length_list_[id] = data_length;
    delete[] data_list_[id];
    data_list_[id] = new UINT8_T[data_length];
    for(int _c = 0; _c < data_length; _c++)
        data_list_[id][_c] = data[_c];

    MakeParam();
    return true;
}
void GroupBulkWrite::ClearParam()
{
    if(ph_->GetProtocolVersion() == 1.0)
        return;

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
int GroupBulkWrite::TxPacket()
{
    if(ph_->GetProtocolVersion() == 1.0 || id_list_.size() == 0)
        return COMM_NOT_AVAILABLE;

    return ph_->BulkWriteTxOnly(port_, param_, param_length_);
}
