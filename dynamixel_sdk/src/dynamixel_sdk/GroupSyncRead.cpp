/*
 * GroupSyncRead.cpp
 *
 *  Created on: 2016. 2. 2.
 *      Author: zerom, leon
 */
#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include <algorithm>
#include "dynamixel_sdk/GroupSyncRead.h"

using namespace ROBOTIS;

GroupSyncRead::GroupSyncRead(PortHandler *port, PacketHandler *ph, UINT16_T start_address, UINT16_T data_length)
    : port_(port),
      ph_(ph),
      last_result_(false),
      is_param_changed_(false),
      param_(0),
      start_address_(start_address),
      data_length_(data_length)
{
    ClearParam();
}

void GroupSyncRead::MakeParam()
{
    if(ph_->GetProtocolVersion() == 1.0 || id_list_.size() == 0)
        return;

    if(param_ != 0)
        delete[] param_;
    param_ = 0;

    param_ = new UINT8_T[id_list_.size() * 1];  // ID(1)

    int _idx = 0;
    for(unsigned int _i = 0; _i < id_list_.size(); _i++)
        param_[_idx++] = id_list_[_i];
}

bool GroupSyncRead::AddParam(UINT8_T id)
{
    if(ph_->GetProtocolVersion() == 1.0)
        return false;

    if(std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
        return false;

    id_list_.push_back(id);
    data_list_[id] = new UINT8_T[data_length_];

    is_param_changed_   = true;
    return true;
}
void GroupSyncRead::RemoveParam(UINT8_T id)
{
    if(ph_->GetProtocolVersion() == 1.0)
        return;

    std::vector<UINT8_T>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
    if(it == id_list_.end())    // NOT exist
        return;

    id_list_.erase(it);
    delete[] data_list_[id];
    data_list_.erase(id);

    is_param_changed_   = true;
}
void GroupSyncRead::ClearParam()
{
    if(ph_->GetProtocolVersion() == 1.0)
        return;

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

int GroupSyncRead::TxPacket()
{
    if(ph_->GetProtocolVersion() == 1.0 || id_list_.size() == 0)
        return COMM_NOT_AVAILABLE;

    if(is_param_changed_ == true)
        MakeParam();

    return ph_->SyncReadTx(port_, start_address_, data_length_, param_, (UINT16_T)id_list_.size() * 1);
}

int GroupSyncRead::RxPacket()
{
    last_result_ = false;

    if(ph_->GetProtocolVersion() == 1.0)
        return COMM_NOT_AVAILABLE;

    int _cnt            = id_list_.size();
    int _result         = COMM_RX_FAIL;

    if(_cnt == 0)
        return COMM_NOT_AVAILABLE;

    for(int _i = 0; _i < _cnt; _i++)
    {
        UINT8_T _id = id_list_[_i];

        _result = ph_->ReadRx(port_, data_length_, data_list_[_id]);
        if(_result != COMM_SUCCESS)
            return _result;
    }

    if(_result == COMM_SUCCESS)
        last_result_ = true;

    return _result;
}

int GroupSyncRead::TxRxPacket()
{
    if(ph_->GetProtocolVersion() == 1.0)
        return COMM_NOT_AVAILABLE;

    int _result         = COMM_TX_FAIL;

    _result = TxPacket();
    if(_result != COMM_SUCCESS)
        return _result;

    return RxPacket();
}

bool GroupSyncRead::IsAvailable(UINT8_T id, UINT16_T address, UINT16_T data_length)
{
    if(ph_->GetProtocolVersion() == 1.0 || last_result_ == false || data_list_.find(id) == data_list_.end())
        return false;

    if(address < start_address_ || start_address_ + data_length_ - data_length < address)
        return false;

    return true;
}

UINT32_T GroupSyncRead::GetData(UINT8_T id, UINT16_T address, UINT16_T data_length)
{
    if(IsAvailable(id, address, data_length) == false)
        return 0;

    switch(data_length)
    {
    case 1:
        return data_list_[id][address - start_address_];

    case 2:
        return DXL_MAKEWORD(data_list_[id][address - start_address_], data_list_[id][address - start_address_ + 1]);

    case 4:
        return DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][address - start_address_ + 0], data_list_[id][address - start_address_ + 1]),
                             DXL_MAKEWORD(data_list_[id][address - start_address_ + 2], data_list_[id][address - start_address_ + 3]));

    default:
        return 0;
    }
}
