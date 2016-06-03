/*
 * Protocol1PacketHandler.h
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom, leon
 */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_


#include "PacketHandler.h"

namespace ROBOTIS
{

class WINDECLSPEC Protocol1PacketHandler : public PacketHandler
{
private:
    static Protocol1PacketHandler *unique_instance_;

    Protocol1PacketHandler();

public:
    static Protocol1PacketHandler *GetInstance() { return unique_instance_; }

    virtual ~Protocol1PacketHandler() { }

    float   GetProtocolVersion() { return 1.0; }

    void    PrintTxRxResult(int result);
    void    PrintRxPacketError(UINT8_T error);

    int TxPacket        (PortHandler *port, UINT8_T *txpacket);
    int RxPacket        (PortHandler *port, UINT8_T *rxpacket);
    int TxRxPacket      (PortHandler *port, UINT8_T *txpacket, UINT8_T *rxpacket, UINT8_T *error = 0);

    int Ping            (PortHandler *port, UINT8_T id, UINT8_T *error = 0);
    int Ping            (PortHandler *port, UINT8_T id, UINT16_T *model_number, UINT8_T *error = 0);

    // BroadcastPing
    int BroadcastPing   (PortHandler *port, std::vector<UINT8_T> &id_list);

    int Action          (PortHandler *port, UINT8_T id);
    int Reboot          (PortHandler *port, UINT8_T id, UINT8_T *error = 0);
    int FactoryReset    (PortHandler *port, UINT8_T id, UINT8_T option, UINT8_T *error = 0);


    int ReadTx          (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length);
    int ReadRx          (PortHandler *port, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);
    int ReadTxRx        (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);

    int Read1ByteTx     (PortHandler *port, UINT8_T id, UINT16_T address);
    int Read1ByteRx     (PortHandler *port, UINT8_T *data, UINT8_T *error = 0);
    int Read1ByteTxRx       (PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T *data, UINT8_T *error = 0);

    int Read2ByteTx     (PortHandler *port, UINT8_T id, UINT16_T address);
    int Read2ByteRx     (PortHandler *port, UINT16_T *data, UINT8_T *error = 0);
    int Read2ByteTxRx       (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T *data, UINT8_T *error = 0);

    int Read4ByteTx     (PortHandler *port, UINT8_T id, UINT16_T address);
    int Read4ByteRx     (PortHandler *port, UINT32_T *data, UINT8_T *error = 0);
    int Read4ByteTxRx       (PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T *data, UINT8_T *error = 0);

    int WriteTxOnly     (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data);
    int WriteTxRx           (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);

    int Write1ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data);
    int Write1ByteTxRx      (PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data, UINT8_T *error = 0);

    int Write2ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data);
    int Write2ByteTxRx      (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data, UINT8_T *error = 0);

    int Write4ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data);
    int Write4ByteTxRx      (PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data, UINT8_T *error = 0);

    int RegWriteTxOnly  (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data);
    int RegWriteTxRx        (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);

    int SyncReadTx      (PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length);
    // SyncReadRx   -> GroupSyncRead class
    // SyncReadTxRx -> GroupSyncRead class

    // param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
    int SyncWriteTxOnly (PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length);

    // param : LEN1 ID1 ADDR1 LEN2 ID2 ADDR2 ...
    int BulkReadTx      (PortHandler *port, UINT8_T *param, UINT16_T param_length);
    // BulkReadRx   -> GroupBulkRead class
    // BulkReadTxRx -> GroupBulkRead class

    int BulkWriteTxOnly (PortHandler *port, UINT8_T *param, UINT16_T param_length);
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_ */
