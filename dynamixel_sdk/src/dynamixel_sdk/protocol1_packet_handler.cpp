/*
 * Protocol1PacketHandler.cpp
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom, leon
 */
#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include <string.h>
#include <stdlib.h>
#include "dynamixel_sdk/Protocol1PacketHandler.h"

#define TXPACKET_MAX_LEN    (250)
#define RXPACKET_MAX_LEN    (250)

///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

using namespace ROBOTIS;

Protocol1PacketHandler *Protocol1PacketHandler::unique_instance_ = new Protocol1PacketHandler();

Protocol1PacketHandler::Protocol1PacketHandler() { }

void Protocol1PacketHandler::PrintTxRxResult(int result)
{
    switch(result)
    {
    case COMM_SUCCESS:
        printf("[TxRxResult] Communication success.\n");
        break;

    case COMM_PORT_BUSY:
        printf("[TxRxResult] Port is in use!\n");
        break;

    case COMM_TX_FAIL:
        printf("[TxRxResult] Failed transmit instruction packet!\n");
        break;

    case COMM_RX_FAIL:
        printf("[TxRxResult] Failed get status packet from device!\n");
        break;

    case COMM_TX_ERROR:
        printf("[TxRxResult] Incorrect instruction packet!\n");
        break;

    case COMM_RX_WAITING:
        printf("[TxRxResult] Now recieving status packet!\n");
        break;

    case COMM_RX_TIMEOUT:
        printf("[TxRxResult] There is no status packet!\n");
        break;

    case COMM_RX_CORRUPT:
        printf("[TxRxResult] Incorrect status packet!\n");
        break;

    case COMM_NOT_AVAILABLE:
        printf("[TxRxResult] Protocol does not support This function!\n");
        break;

    default:
        break;
    }
}

void Protocol1PacketHandler::PrintRxPacketError(UINT8_T error)
{
    if(error & ERRBIT_VOLTAGE)
        printf("[RxPacketError] Input voltage error!\n");

    if(error & ERRBIT_ANGLE)
        printf("[RxPacketError] Angle limit error!\n");

    if(error & ERRBIT_OVERHEAT)
        printf("[RxPacketError] Overheat error!\n");

    if(error & ERRBIT_RANGE)
        printf("[RxPacketError] Out of range error!\n");

    if(error & ERRBIT_CHECKSUM)
        printf("[RxPacketError] Checksum error!\n");

    if(error & ERRBIT_OVERLOAD)
        printf("[RxPacketError] Overload error!\n");

    if(error & ERRBIT_INSTRUCTION)
        printf("[RxPacketError] Instruction code error!\n");
}

int Protocol1PacketHandler::TxPacket(PortHandler *port, UINT8_T *txpacket)
{
    UINT8_T _checksum               = 0;
    UINT8_T _total_packet_length    = txpacket[PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LENGTH
    UINT8_T _written_packet_length  = 0;

    if(port->is_using)
        return COMM_PORT_BUSY;
    port->is_using = true;

    // check max packet length
    if(_total_packet_length > TXPACKET_MAX_LEN)
    {
        port->is_using = false;
        return COMM_TX_ERROR;
    }

    // make packet header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;

    // add a checksum to the packet
    for(int _idx = 2; _idx < _total_packet_length - 1; _idx++)   // except header, checksum
        _checksum += txpacket[_idx];
    txpacket[_total_packet_length - 1] = ~_checksum;

    // tx packet
    port->ClearPort();
    _written_packet_length = port->WritePort(txpacket, _total_packet_length);
    if(_total_packet_length != _written_packet_length)
    {
        port->is_using = false;
        return COMM_TX_FAIL;
    }

    return COMM_SUCCESS;
}

int Protocol1PacketHandler::RxPacket(PortHandler *port, UINT8_T *rxpacket)
{
    int     _result         = COMM_TX_FAIL;

    UINT8_T _checksum       = 0;
    UINT8_T _rx_length      = 0;
    UINT8_T _wait_length    = 6;    // minimum length ( HEADER0 HEADER1 ID LENGTH ERROR CHKSUM )

    while(true)
    {
        _rx_length += port->ReadPort(&rxpacket[_rx_length], _wait_length - _rx_length);
        if(_rx_length >= _wait_length)
        {
            UINT8_T _idx = 0;

            // find packet header
            for(_idx = 0; _idx < (_rx_length - 1); _idx++)
            {
                if(rxpacket[_idx] == 0xFF && rxpacket[_idx+1] == 0xFF)
                    break;
            }

            if(_idx == 0)   // found at the beginning of the packet
            {
                if(rxpacket[PKT_ID] > 0xFD ||                   // unavailable ID
                   rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN ||   // unavailable Length
                   rxpacket[PKT_ERROR] >= 0x64)                 // unavailable Error
                {
                    // remove the first byte in the packet
                    for(UINT8_T _s = 0; _s < _rx_length - 1; _s++)
                        rxpacket[_s] = rxpacket[1 + _s];
                    //memcpy(&rxpacket[0], &rxpacket[_idx], _rx_length - _idx);
                    _rx_length -= 1;
                    continue;
                }

                // re-calculate the exact length of the rx packet
                if(_wait_length != rxpacket[PKT_LENGTH] + PKT_LENGTH + 1)
                {
                    _wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
                    continue;
                }
                
                if(_rx_length < _wait_length)
                {
                    // check timeout
                    if(port->IsPacketTimeout() == true)
                    {
                        if(_rx_length == 0)
                            _result = COMM_RX_TIMEOUT;
                        else
                            _result = COMM_RX_CORRUPT;
                        break;
                    }
                    else
                        continue;
                }

                // calculate checksum
                for(int _i = 2; _i < _wait_length - 1; _i++)   // except header, checksum
                    _checksum += rxpacket[_i];
                _checksum = ~_checksum;

                // verify checksum
                if(rxpacket[_wait_length - 1] == _checksum)
                    _result = COMM_SUCCESS;
                else
                    _result = COMM_RX_CORRUPT;
                break;
            }
            else
            {
                // remove unnecessary packets
                for(UINT8_T _s = 0; _s < _rx_length - _idx; _s++)
                    rxpacket[_s] = rxpacket[_idx + _s];
                //memcpy(&rxpacket[0], &rxpacket[_idx], _rx_length - _idx);
                _rx_length -= _idx;
            }
        }
        else
        {
            // check timeout
            if(port->IsPacketTimeout() == true)
            {
                if(_rx_length == 0)
                    _result = COMM_RX_TIMEOUT;
                else
                    _result = COMM_RX_CORRUPT;
                break;
            }
        }
    }
    port->is_using = false;

    return _result;
}

// NOT for BulkRead instruction
int Protocol1PacketHandler::TxRxPacket(PortHandler *port, UINT8_T *txpacket, UINT8_T *rxpacket, UINT8_T *error)
{
    int _result = COMM_TX_FAIL;

    // tx packet
    _result = TxPacket(port, txpacket);
    if(_result != COMM_SUCCESS)
        return _result;

    // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
    // (Instruction == Action) == no need to wait for status packet
    if((txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ) ||
       (txpacket[PKT_INSTRUCTION] == INST_ACTION))
    {
        port->is_using = false;
        return _result;
    }

    // set packet timeout
    if(txpacket[PKT_INSTRUCTION] == INST_READ)
        port->SetPacketTimeout((UINT16_T)(txpacket[PKT_PARAMETER0+1] + 6));
    else
        port->SetPacketTimeout((UINT16_T)6);

    // rx packet
    _result = RxPacket(port, rxpacket);
    // check txpacket ID == rxpacket ID
    if(txpacket[PKT_ID] != rxpacket[PKT_ID])
        _result = RxPacket(port, rxpacket);

    if(_result == COMM_SUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
    {
        if(error != 0)
            *error = (UINT8_T)rxpacket[PKT_ERROR];
    }

    return _result;
}

int Protocol1PacketHandler::Ping(PortHandler *port, UINT8_T id, UINT8_T *error)
{
    return Ping(port, id, 0, error);
}

int Protocol1PacketHandler::Ping(PortHandler *port, UINT8_T id, UINT16_T *model_number, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T txpacket[6]         = {0};
    UINT8_T rxpacket[6]         = {0};

    if(id >= BROADCAST_ID)
        return COMM_NOT_AVAILABLE;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = 2;
    txpacket[PKT_INSTRUCTION]   = INST_PING;

    _result = TxRxPacket(port, txpacket, rxpacket, error);
    if(_result == COMM_SUCCESS && model_number != 0)
    {
        UINT8_T _data[2] = {0};
        _result = ReadTxRx(port, id, 0, 2, _data);  // Address 0 : Model Number
        if(_result == COMM_SUCCESS)
            *model_number = DXL_MAKEWORD(_data[0], _data[1]);
    }

    return _result;
}

int Protocol1PacketHandler::BroadcastPing(PortHandler *port, std::vector<UINT8_T> &id_list)
{
    return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::Action(PortHandler *port, UINT8_T id)
{
    UINT8_T txpacket[6]         = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = 2;
    txpacket[PKT_INSTRUCTION]   = INST_ACTION;

    return TxRxPacket(port, txpacket, 0);
}

int Protocol1PacketHandler::Reboot(PortHandler *port, UINT8_T id, UINT8_T *error)
{
    return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::FactoryReset(PortHandler *port, UINT8_T id, UINT8_T option, UINT8_T *error)
{
    UINT8_T txpacket[6]         = {0};
    UINT8_T rxpacket[6]         = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = 2;
    txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET;

    return TxRxPacket(port, txpacket, rxpacket, error);
}

int Protocol1PacketHandler::ReadTx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T txpacket[8]         = {0};

    if(id >= BROADCAST_ID)
        return COMM_NOT_AVAILABLE;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = 4;
    txpacket[PKT_INSTRUCTION]   = INST_READ;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)address;
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)length;

    _result = TxPacket(port, txpacket);

    // set packet timeout
    if(_result == COMM_SUCCESS)
        port->SetPacketTimeout((UINT16_T)(length+6));

    return _result;
}

int Protocol1PacketHandler::ReadRx(PortHandler *port, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;
    UINT8_T *rxpacket           = (UINT8_T *)malloc(RXPACKET_MAX_LEN);//(length+6);
    //UINT8_T *rxpacket           = new UINT8_T[length+6];

    _result = RxPacket(port, rxpacket);
    if(_result == COMM_SUCCESS)
    {
        if(error != 0)
            *error = (UINT8_T)rxpacket[PKT_ERROR];
        for(UINT8_T _s = 0; _s < length; _s++)
            data[_s] = rxpacket[PKT_PARAMETER0 + _s];
        //memcpy(data, &rxpacket[PKT_PARAMETER0], length);
    }

    free(rxpacket);
    //delete[] rxpacket;
    return _result;
}

int Protocol1PacketHandler::ReadTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result = COMM_TX_FAIL;

    UINT8_T txpacket[8]         = {0};
    UINT8_T *rxpacket           = (UINT8_T *)malloc(RXPACKET_MAX_LEN);//(length+6);

    if(id >= BROADCAST_ID)
        return COMM_NOT_AVAILABLE;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = 4;
    txpacket[PKT_INSTRUCTION]   = INST_READ;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)address;
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)length;

    _result = TxRxPacket(port, txpacket, rxpacket, error);
    if(_result == COMM_SUCCESS)
    {
        if(error != 0)
            *error = (UINT8_T)rxpacket[PKT_ERROR];
        for(UINT8_T _s = 0; _s < length; _s++)
            data[_s] = rxpacket[PKT_PARAMETER0 + _s];
        //memcpy(data, &rxpacket[PKT_PARAMETER0], length);
    }

    free(rxpacket);
    //delete[] rxpacket;
    return _result;
}

int Protocol1PacketHandler::Read1ByteTx(PortHandler *port, UINT8_T id, UINT16_T address)
{
    return ReadTx(port, id, address, 1);
}
int Protocol1PacketHandler::Read1ByteRx(PortHandler *port, UINT8_T *data, UINT8_T *error)
{
    UINT8_T _data[1] = {0};
    int _result = ReadRx(port, 1, _data, error);
    if(_result == COMM_SUCCESS)
        *data = _data[0];
    return _result;
}
int Protocol1PacketHandler::Read1ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T *data, UINT8_T *error)
{
    UINT8_T _data[1] = {0};
    int _result = ReadTxRx(port, id, address, 1, _data, error);
    if(_result == COMM_SUCCESS)
        *data = _data[0];
    return _result;
}

int Protocol1PacketHandler::Read2ByteTx(PortHandler *port, UINT8_T id, UINT16_T address)
{
    return ReadTx(port, id, address, 2);
}
int Protocol1PacketHandler::Read2ByteRx(PortHandler *port, UINT16_T *data, UINT8_T *error)
{
    UINT8_T _data[2] = {0};
    int _result = ReadRx(port, 2, _data, error);
    if(_result == COMM_SUCCESS)
        *data = DXL_MAKEWORD(_data[0], _data[1]);
    return _result;
}
int Protocol1PacketHandler::Read2ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T *data, UINT8_T *error)
{
    UINT8_T _data[2] = {0};
    int _result = ReadTxRx(port, id, address, 2, _data, error);
    if(_result == COMM_SUCCESS)
        *data = DXL_MAKEWORD(_data[0], _data[1]);
    return _result;
}

int Protocol1PacketHandler::Read4ByteTx(PortHandler *port, UINT8_T id, UINT16_T address)
{
    return COMM_NOT_AVAILABLE;
}
int Protocol1PacketHandler::Read4ByteRx(PortHandler *port, UINT32_T *data, UINT8_T *error)
{
    return COMM_NOT_AVAILABLE;
}
int Protocol1PacketHandler::Read4ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T *data, UINT8_T *error)
{
    return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::WriteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length+7);
    //UINT8_T *txpacket           = new UINT8_T[length+7];

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = length+3;
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0]    = (UINT8_T)address;

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+1+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

    _result = TxPacket(port, txpacket);
    port->is_using = false;

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol1PacketHandler::WriteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length+6);
    //UINT8_T *txpacket           = new UINT8_T[length+6];
    UINT8_T rxpacket[6]         = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = length+3;
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0]    = (UINT8_T)address;

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+1+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

    _result = TxRxPacket(port, txpacket, rxpacket, error);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol1PacketHandler::Write1ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data)
{
    UINT8_T _data[1] = { data };
    return WriteTxOnly(port, id, address, 1, _data);
}
int Protocol1PacketHandler::Write1ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data, UINT8_T *error)
{
    UINT8_T _data[1] = { data };
    return WriteTxRx(port, id, address, 1, _data, error);
}

int Protocol1PacketHandler::Write2ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data)
{
    UINT8_T _data[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
    return WriteTxOnly(port, id, address, 2, _data);
}
int Protocol1PacketHandler::Write2ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data, UINT8_T *error)
{
    UINT8_T _data[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
    return WriteTxRx(port, id, address, 2, _data, error);
}

int Protocol1PacketHandler::Write4ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data)
{
    return COMM_NOT_AVAILABLE;
}
int Protocol1PacketHandler::Write4ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data, UINT8_T *error)
{
    return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::RegWriteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length+6);
    //UINT8_T *txpacket           = new UINT8_T[length+6];

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = length+3;
    txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE;
    txpacket[PKT_PARAMETER0]    = (UINT8_T)address;

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+1+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

    _result = TxPacket(port, txpacket);
    port->is_using = false;

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol1PacketHandler::RegWriteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length+6);
    //UINT8_T *txpacket           = new UINT8_T[length+6];
    UINT8_T rxpacket[6]         = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = length+3;
    txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE;
    txpacket[PKT_PARAMETER0]    = (UINT8_T)address;

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+1+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

    _result = TxRxPacket(port, txpacket, rxpacket, error);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol1PacketHandler::SyncReadTx(PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length)
{
    return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::SyncWriteTxOnly(PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(param_length+8);    // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM
    //UINT8_T *txpacket           = new UINT8_T[param_length + 8];    // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH]        = param_length + 4; // 4: INST START_ADDR DATA_LEN ... CHKSUM
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0+0]  = start_address;
    txpacket[PKT_PARAMETER0+1]  = data_length;

    for(UINT8_T _s = 0; _s < param_length; _s++)
        txpacket[PKT_PARAMETER0+2+_s] = param[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+2], param, param_length);

    _result = TxRxPacket(port, txpacket, 0, 0);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol1PacketHandler::BulkReadTx(PortHandler *port, UINT8_T *param, UINT16_T param_length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(param_length+7);    // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM
    //UINT8_T *txpacket           = new UINT8_T[param_length + 7];    // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH]        = param_length + 3; // 3: INST 0x00 ... CHKSUM
    txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;
    txpacket[PKT_PARAMETER0+0]  = 0x00;

    for(UINT8_T _s = 0; _s < param_length; _s++)
        txpacket[PKT_PARAMETER0+1+_s] = param[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+1], param, param_length);

    _result = TxPacket(port, txpacket);
    if(_result == COMM_SUCCESS)
    {
        int _wait_length = 0;
        for(int _i = 0; _i < param_length; _i += 3)
            _wait_length += param[_i] + 7;
        port->SetPacketTimeout((UINT16_T)_wait_length);
    }

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol1PacketHandler::BulkWriteTxOnly(PortHandler *port, UINT8_T *param, UINT16_T param_length)
{
    return COMM_NOT_AVAILABLE;
}
