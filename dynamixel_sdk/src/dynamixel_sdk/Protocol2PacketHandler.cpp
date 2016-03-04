/*
 * Protocol2PacketHandler.cpp
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dynamixel_sdk/Protocol2PacketHandler.h"

#define TXPACKET_MAX_LEN    (4*1024)
#define RXPACKET_MAX_LEN    (4*1024)

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error

#define ERRBIT_ALERT            128     //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

using namespace ROBOTIS;

Protocol2PacketHandler *Protocol2PacketHandler::unique_instance_ = new Protocol2PacketHandler();

Protocol2PacketHandler::Protocol2PacketHandler() { }

unsigned short Protocol2PacketHandler::UpdateCRC(UINT16_T crc_accum, UINT8_T *data_blk_ptr, UINT16_T data_blk_size)
{
    UINT16_T i, j;
    UINT16_T crc_table[256] = {0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((UINT16_T)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void Protocol2PacketHandler::AddStuffing(UINT8_T *packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
    UINT8_T temp[TXPACKET_MAX_LEN] = {0};

    for(UINT8_T _s = PKT_HEADER0; _s <= PKT_LENGTH_H; _s++)
        temp[_s] = packet[_s]; // FF FF FD XX ID LEN_L LEN_H
    //memcpy(temp, packet, PKT_LENGTH_H+1);
    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        temp[index++] = packet[i+PKT_INSTRUCTION];
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD
            temp[index++] = 0xFD;
            packet_length_out++;
        }
    }
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];


    //////////////////////////
    if(packet_length_in != packet_length_out)
        packet = (UINT8_T *)realloc(packet, index * sizeof(UINT8_T));

    ///////////////////////////

    for(UINT8_T _s = 0; _s < index; _s++)
        packet[_s] = temp[_s];
    //memcpy(packet, temp, index);
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void Protocol2PacketHandler::RemoveStuffing(UINT8_T *packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    index = PKT_INSTRUCTION;
    for( i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        if(packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD FD
            packet_length_out--;
            i++;
        }
        packet[index++] = packet[i+PKT_INSTRUCTION];
    }
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

int Protocol2PacketHandler::TxPacket(PortHandler *port, UINT8_T *txpacket)
{
    UINT16_T _total_packet_length   = 0;
    UINT16_T _written_packet_length = 0;

    if(port->is_using)
        return COMM_PORT_BUSY;
    port->is_using = true;

    // byte stuffing for header
    AddStuffing(txpacket);

    // check max packet length
    _total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;
                           // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
    if(_total_packet_length > TXPACKET_MAX_LEN)
    {
        port->is_using = false;
        return COMM_TX_ERROR;
    }

    // make packet header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]  = 0x00;

    // add CRC16
    UINT16_T crc = UpdateCRC(0, txpacket, _total_packet_length - 2);    // 2: CRC16
    txpacket[_total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[_total_packet_length - 1] = DXL_HIBYTE(crc);

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

int Protocol2PacketHandler::RxPacket(PortHandler *port, UINT8_T *rxpacket)
{
    int     _result         = COMM_TX_FAIL;

    UINT16_T _rx_length     = 0;
    UINT16_T _wait_length   = 11;
            // minimum length ( HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H )

    while(true)
    {
        _rx_length += port->ReadPort(&rxpacket[_rx_length], _wait_length - _rx_length);
        if(_rx_length >= _wait_length)
        {
            UINT16_T _idx = 0;

            // find packet header
//            for(_idx = 0; _idx < (_rx_length - 2); _idx++)
//            {
//                if(rxpacket[_idx] == 0xFF && rxpacket[_idx+1] == 0xFF && rxpacket[_idx+2] == 0xFD)
//                    break;
//            }
            for(_idx = 0; _idx < (_rx_length - 3); _idx++)
            {
                if((rxpacket[_idx] == 0xFF) && (rxpacket[_idx+1] == 0xFF) && (rxpacket[_idx+2] == 0xFD) && (rxpacket[_idx+3] != 0xFD))
                    break;
            }

            if(_idx == 0)   // found at the beginning of the packet
            {
                // re-calculate the exact length of the rx packet
                _wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
                if(_wait_length > 100)
                	fprintf(stderr, "id : %d , _wait_length : %d \n", rxpacket[PKT_ID], _wait_length);

				if(_wait_length > RXPACKET_MAX_LEN - 10)
				{
					_result = COMM_RX_CORRUPT;
				    break;
				}

                if(_rx_length < _wait_length)
                {
                    // check timeout
                    if(port->IsPacketTimeout() == true)
                    {
                        if(_rx_length == 0)
                            _result = COMM_RX_TIMEOUT;
                        else {
                            _result = COMM_RX_CORRUPT;
//                        	fprintf(stderr, "Fail Rx Packet(short_length1) : ");
//                        	for(int _i = 0; _i < _rx_length + 11; _i++)
//                        	{
//                        		fprintf(stderr, " %x", rxpacket[_i]);
//                        	}
//                        	fprintf(stderr, "\n");
                        }
                        break;
                    }

                    else
                        continue;
                }

                // verify CRC16
                UINT16_T crc = DXL_MAKEWORD(rxpacket[_wait_length-2], rxpacket[_wait_length-1]);
                if(UpdateCRC(0, rxpacket, _wait_length - 2) == crc)
                    _result = COMM_SUCCESS;
                else {
                    _result = COMM_RX_CORRUPT;
//                	fprintf(stderr, "Rx Packet(crc_error) : ");
//                	for(int _i = 0; _i < _rx_length + 11; _i++)
//                	{
//                		fprintf(stderr, " %x", rxpacket[_i]);
//                	}
//                	fprintf(stderr, "\n");
                }
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
                else {
                    _result = COMM_RX_CORRUPT;
//                	fprintf(stderr, "Fail Rx Packet(short_length2): ");
//                	for(int _i = 0; _i < _rx_length + 11; _i++)
//                	{
//                		fprintf(stderr, " %x", rxpacket[_i]);
//                	}
//                	fprintf(stderr, "\n");
                }
                break;
            }
        }
    }
    port->is_using = false;

    if(_result == COMM_SUCCESS)
        RemoveStuffing(rxpacket);

    return _result;
}

// NOT for BulkRead / SyncRead instruction
int Protocol2PacketHandler::TxRxPacket(PortHandler *port, UINT8_T *txpacket, UINT8_T *rxpacket, UINT8_T *error)
{
    int _result = COMM_TX_FAIL;

    // tx packet
    _result = TxPacket(port, txpacket);
    if(_result != COMM_SUCCESS)
        return _result;

    // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
    // (Instruction == Action) == no need to wait for status packet
    if((txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_BULK_READ) ||
       (txpacket[PKT_ID] == BROADCAST_ID && txpacket[PKT_INSTRUCTION] != INST_SYNC_READ) ||
       (txpacket[PKT_INSTRUCTION] == INST_ACTION))
    {
        port->is_using = false;
        return _result;
    }

    // set packet timeout
    if(txpacket[PKT_INSTRUCTION] == INST_READ)
        port->SetPacketTimeout((UINT16_T)(DXL_MAKEWORD(txpacket[PKT_PARAMETER0+2], txpacket[PKT_PARAMETER0+3]) + 11));
    else
        port->SetPacketTimeout((UINT16_T)11);   // HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H

    // rx packet
    _result = RxPacket(port, rxpacket);
    // TODO: check txpacket ID == rxpacket ID

    if(_result == COMM_SUCCESS && txpacket[PKT_ID] != BROADCAST_ID)
    {
        if(error != 0)
            *error = (UINT8_T)rxpacket[PKT_ERROR];
    }

    return _result;
}

int Protocol2PacketHandler::Ping(PortHandler *port, UINT8_T id, UINT8_T *error)
{
    return Ping(port, id, 0, error);
}

int Protocol2PacketHandler::Ping(PortHandler *port, UINT8_T id, UINT16_T *model_number, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T txpacket[10]        = {0};
    UINT8_T rxpacket[14]        = {0};

    if(id >= BROADCAST_ID)
        return COMM_NOT_AVAILABLE;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = 3;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_PING;

    _result = TxRxPacket(port, txpacket, rxpacket, error);
    if(_result == COMM_SUCCESS && model_number != 0)
        *model_number = DXL_MAKEWORD(rxpacket[PKT_PARAMETER0+1], rxpacket[PKT_PARAMETER0+2]);

    return _result;
}

int Protocol2PacketHandler::BroadcastPing(PortHandler *port, std::vector<UINT8_T> &id_list)
{
    const int STATUS_LENGTH     = 14;
    int _result                 = COMM_TX_FAIL;

    id_list.clear();

    UINT16_T _rx_length         = 0;
    UINT16_T _wait_length       = STATUS_LENGTH * MAX_ID;

    UINT8_T txpacket[10]        = {0};
    UINT8_T rxpacket[STATUS_LENGTH * MAX_ID] = {0};

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = 3;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_PING;

    _result = TxPacket(port, txpacket);
    if(_result != COMM_SUCCESS)
    {
        port->is_using = false;
        return _result;
    }

    // set rx timeout
    port->SetPacketTimeout((UINT16_T)(_wait_length * 30));

    while(1)
    {
        _rx_length += port->ReadPort(&rxpacket[_rx_length], _wait_length - _rx_length);
        if(port->IsPacketTimeout() == true)// || _rx_length >= _wait_length)
            break;
    }

    port->is_using = false;

    if(_rx_length == 0)
        return COMM_RX_TIMEOUT;

    while(1)
    {
        if(_rx_length < STATUS_LENGTH)
            return COMM_RX_CORRUPT;

        UINT16_T _idx = 0;

        // find packet header
        for(_idx = 0; _idx < (_rx_length - 2); _idx++)
        {
            if(rxpacket[_idx] == 0xFF && rxpacket[_idx+1] == 0xFF && rxpacket[_idx+2] == 0xFD)
                break;
        }

        if(_idx == 0)   // found at the beginning of the packet
        {
            // verify CRC16
            UINT16_T crc = DXL_MAKEWORD(rxpacket[STATUS_LENGTH-2], rxpacket[STATUS_LENGTH-1]);

            if(UpdateCRC(0, rxpacket, STATUS_LENGTH - 2) == crc)
            {
                _result = COMM_SUCCESS;

                id_list.push_back(rxpacket[PKT_ID]);

                for(UINT8_T _s = 0; _s < _rx_length - STATUS_LENGTH; _s++)
                    rxpacket[_s] = rxpacket[STATUS_LENGTH + _s];
                _rx_length -= STATUS_LENGTH;

                if(_rx_length == 0)
                    return _result;
            }
            else
            {
                _result = COMM_RX_CORRUPT;

                // remove header (0xFF 0xFF 0xFD)
                for(UINT8_T _s = 0; _s < _rx_length - 3; _s++)
                    rxpacket[_s] = rxpacket[3 + _s];
                _rx_length -= 3;
            }
        }
        else
        {
            // remove unnecessary packets
            for(UINT8_T _s = 0; _s < _rx_length - _idx; _s++)
                rxpacket[_s] = rxpacket[_idx + _s];
            _rx_length -= _idx;
        }
    }

    return _result;
}

int Protocol2PacketHandler::Action(PortHandler *port, UINT8_T id)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T txpacket[10]        = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = 3;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_ACTION;

    return TxRxPacket(port, txpacket, 0);
}

int Protocol2PacketHandler::Reboot(PortHandler *port, UINT8_T id, UINT8_T *error)
{
    UINT8_T txpacket[10]        = {0};
    UINT8_T rxpacket[11]        = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = 3;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_REBOOT;

    return TxRxPacket(port, txpacket, rxpacket, error);
}

int Protocol2PacketHandler::FactoryReset(PortHandler *port, UINT8_T id, UINT8_T option, UINT8_T *error)
{
    UINT8_T txpacket[11]        = {0};
    UINT8_T rxpacket[11]        = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = 4;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET;
    txpacket[PKT_PARAMETER0]    = option;

    return TxRxPacket(port, txpacket, rxpacket, error);
}

int Protocol2PacketHandler::ReadTx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T txpacket[14]        = {0};

    if(id >= BROADCAST_ID)
        return COMM_NOT_AVAILABLE;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = 7;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_READ;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)DXL_HIBYTE(address);
    txpacket[PKT_PARAMETER0+2]  = (UINT8_T)DXL_LOBYTE(length);
    txpacket[PKT_PARAMETER0+3]  = (UINT8_T)DXL_HIBYTE(length);

    _result = TxPacket(port, txpacket);

    // set packet timeout
    if(_result == COMM_SUCCESS)
        port->SetPacketTimeout((UINT16_T)(length + 11));

    return _result;
}

int Protocol2PacketHandler::ReadRx(PortHandler *port, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;
    //UINT8_T *rxpacket           = (UINT8_T *)malloc(RXPACKET_MAX_LEN);//(length + 11 + (length/3));  // (length/3): consider stuffing
    UINT8_T *rxpacket           = ( UINT8_T* )calloc(RXPACKET_MAX_LEN, sizeof(UINT8_T));//(length + 11 + (length/3));  // (length/3): consider stuffing
    //UINT8_T *rxpacket           = new UINT8_T[length + 11 + (length/3)];    // (length/3): consider stuffing

    _result = RxPacket(port, rxpacket);
    if(_result == COMM_SUCCESS)
    {
        if(error != 0)
            *error = (UINT8_T)rxpacket[PKT_ERROR];
        for(UINT8_T _s = 0; _s < length; _s++)
            data[_s] = rxpacket[PKT_PARAMETER0 + 1 + _s];
        //memcpy(data, &rxpacket[PKT_PARAMETER0+1], length);
    }
//    else
//    {
//    	fprintf(stderr, "Fail Rx Packet : ");
//    	for(int _i = 0; _i < length + 11; _i++)
//    	{
//    		fprintf(stderr, " %x", rxpacket[_i]);
//    	}
//    	fprintf(stderr, "\n");
//    }

    free(rxpacket);
    //delete[] rxpacket;
    return _result;
}

int Protocol2PacketHandler::ReadTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    _result = ReadTx(port, id, address, length);
    if(_result != COMM_SUCCESS)
        return _result;

    return ReadRx(port, length, data, error);
}

int Protocol2PacketHandler::Read1ByteTx(PortHandler *port, UINT8_T id, UINT16_T address)
{
    return ReadTx(port, id, address, 1);
}
int Protocol2PacketHandler::Read1ByteRx(PortHandler *port, UINT8_T *data, UINT8_T *error)
{
    UINT8_T _data[1] = {0};
    int _result = ReadRx(port, 1, _data, error);
    *data = _data[0];
    return _result;
}
int Protocol2PacketHandler::Read1ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T *data, UINT8_T *error)
{
    int _result = COMM_TX_FAIL;

    _result = Read1ByteTx(port, id, address);
    if(_result != COMM_SUCCESS)
        return _result;

    return Read1ByteRx(port, data, error);
}

int Protocol2PacketHandler::Read2ByteTx(PortHandler *port, UINT8_T id, UINT16_T address)
{
    return ReadTx(port, id, address, 2);
}
int Protocol2PacketHandler::Read2ByteRx(PortHandler *port, UINT16_T *data, UINT8_T *error)
{
    UINT8_T _data[2] = {0};
    int _result = ReadRx(port, 2, _data, error);
    *data = DXL_MAKEWORD(_data[0], _data[1]);
    return _result;
}
int Protocol2PacketHandler::Read2ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T *data, UINT8_T *error)
{
    int _result = COMM_TX_FAIL;

    _result = Read2ByteTx(port, id, address);
    if(_result != COMM_SUCCESS)
        return _result;

    return Read2ByteRx(port, data, error);
}

int Protocol2PacketHandler::Read4ByteTx(PortHandler *port, UINT8_T id, UINT16_T address)
{
    return ReadTx(port, id, address, 4);
}
int Protocol2PacketHandler::Read4ByteRx(PortHandler *port, UINT32_T *data, UINT8_T *error)
{
    UINT8_T _data[4] = {0};
    int _result = ReadRx(port, 4, _data, error);
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(_data[0], _data[1]), DXL_MAKEWORD(_data[2], _data[3]));
    return _result;
}
int Protocol2PacketHandler::Read4ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T *data, UINT8_T *error)
{
    int _result = COMM_TX_FAIL;

    _result = Read4ByteTx(port, id, address);
    if(_result != COMM_SUCCESS)
        return _result;

    return Read4ByteRx(port, data, error);
}


int Protocol2PacketHandler::WriteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length+12);
    //UINT8_T *txpacket           = new UINT8_T[length+12];

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)DXL_HIBYTE(address);

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+2+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

    _result = TxPacket(port, txpacket);
    port->is_using = false;

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol2PacketHandler::WriteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length + 12);
    //UINT8_T *txpacket           = new UINT8_T[length+12];
    UINT8_T rxpacket[11]        = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)DXL_HIBYTE(address);

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+2+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

    _result = TxRxPacket(port, txpacket, rxpacket, error);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol2PacketHandler::Write1ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data)
{
    UINT8_T _data[1] = { data };
    return WriteTxOnly(port, id, address, 1, _data);
}
int Protocol2PacketHandler::Write1ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data, UINT8_T *error)
{
    UINT8_T _data[1] = { data };
    return WriteTxRx(port, id, address, 1, _data, error);
}

int Protocol2PacketHandler::Write2ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data)
{
    UINT8_T _data[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
    return WriteTxOnly(port, id, address, 2, _data);
}
int Protocol2PacketHandler::Write2ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data, UINT8_T *error)
{
    UINT8_T _data[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
    return WriteTxRx(port, id, address, 2, _data, error);
}

int Protocol2PacketHandler::Write4ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data)
{
    UINT8_T _data[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
    return WriteTxOnly(port, id, address, 4, _data);
}
int Protocol2PacketHandler::Write4ByteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data, UINT8_T *error)
{
    UINT8_T _data[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
    return WriteTxRx(port, id, address, 4, _data, error);
}

int Protocol2PacketHandler::RegWriteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length + 12);
    //UINT8_T *txpacket           = new UINT8_T[length+12];

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
    txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)DXL_HIBYTE(address);

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+2+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

    _result = TxPacket(port, txpacket);
    port->is_using = false;

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol2PacketHandler::RegWriteTxRx(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(length + 12);
    //UINT8_T *txpacket           = new UINT8_T[length+12];
    UINT8_T rxpacket[11]        = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
    txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE;
    txpacket[PKT_PARAMETER0+0]  = (UINT8_T)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (UINT8_T)DXL_HIBYTE(address);

    for(UINT8_T _s = 0; _s < length; _s++)
        txpacket[PKT_PARAMETER0+2+_s] = data[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

    _result = TxRxPacket(port, txpacket, rxpacket, error);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol2PacketHandler::SyncReadTx(PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(param_length + 14);
            // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_READ;
    txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address);
    txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address);
    txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
    txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);

    for(UINT8_T _s = 0; _s < param_length; _s++)
        txpacket[PKT_PARAMETER0+4+_s] = param[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

    _result = TxPacket(port, txpacket);
    if(_result == COMM_SUCCESS)
        port->SetPacketTimeout((UINT16_T)((11 + data_length) * param_length));

    free(txpacket);
    return _result;
}

int Protocol2PacketHandler::SyncWriteTxOnly(PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(param_length + 14);
    //UINT8_T *txpacket           = new UINT8_T[param_length + 14];
            // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address);
    txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address);
    txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
    txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);

    for(UINT8_T _s = 0; _s < param_length; _s++)
        txpacket[PKT_PARAMETER0+4+_s] = param[_s];
    //memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

    _result = TxRxPacket(port, txpacket, 0, 0);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol2PacketHandler::BulkReadTx(PortHandler *port, UINT8_T *param, UINT16_T param_length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(param_length + 10);
    //UINT8_T *txpacket           = new UINT8_T[param_length + 10];
            // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
    txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;

    for(UINT8_T _s = 0; _s < param_length; _s++)
        txpacket[PKT_PARAMETER0+_s] = param[_s];
    //memcpy(&txpacket[PKT_PARAMETER0], param, param_length);

    _result = TxPacket(port, txpacket);
    if(_result == COMM_SUCCESS)
    {
        int _wait_length = 0;
        for(int _i = 0; _i < param_length; _i += 5)
            _wait_length += DXL_MAKEWORD(param[_i+3], param[_i+4]) + 10;
        port->SetPacketTimeout((UINT16_T)_wait_length);
    }

    free(txpacket);
    //delete[] txpacket;
    return _result;
}

int Protocol2PacketHandler::BulkWriteTxOnly(PortHandler *port, UINT8_T *param, UINT16_T param_length)
{
    int _result                 = COMM_TX_FAIL;

    UINT8_T *txpacket           = (UINT8_T *)malloc(param_length + 10);
    //UINT8_T *txpacket           = new UINT8_T[param_length + 10];
            // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
    txpacket[PKT_INSTRUCTION]   = INST_BULK_WRITE;

    for(UINT8_T _s = 0; _s < param_length; _s++)
        txpacket[PKT_PARAMETER0+_s] = param[_s];
    //memcpy(&txpacket[PKT_PARAMETER0], param, param_length);

    _result = TxRxPacket(port, txpacket, 0, 0);

    free(txpacket);
    //delete[] txpacket;
    return _result;
}
