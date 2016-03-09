/*
 * PacketHandler.h
 *
 *  Created on: 2016. 1. 26.
 *      Author: zerom
 */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_


#include <stdio.h>
#include <vector>
#include "RobotisDef.h"
#include "PortHandler.h"

#define BROADCAST_ID        0xFE    // 254
#define MAX_ID              0xFC    // 252

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

/* Instruction for DXL Protocol */
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT             8
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

// Communication Result
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_PORT_BUSY      -1000   // Port is busy (in use)
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   //

namespace ROBOTIS
{

class PacketHandler
{
protected:
    PacketHandler() { }

public:
    static PacketHandler *GetPacketHandler(float protocol_version = 2.0);

    virtual ~PacketHandler() { }

    virtual float   GetProtocolVersion() = 0;

    virtual void    PrintTxRxResult(int result) = 0;
    virtual void    PrintRxPacketError(UINT8_T error) = 0;

    virtual int TxPacket        (PortHandler *port, UINT8_T *txpacket) = 0;
    virtual int RxPacket        (PortHandler *port, UINT8_T *rxpacket) = 0;
    virtual int TxRxPacket      (PortHandler *port, UINT8_T *txpacket, UINT8_T *rxpacket, UINT8_T *error = 0) = 0;

    virtual int Ping            (PortHandler *port, UINT8_T id, UINT8_T *error = 0) = 0;
    virtual int Ping            (PortHandler *port, UINT8_T id, UINT16_T *model_number, UINT8_T *error = 0) = 0;

    // BroadcastPing
    virtual int BroadcastPing   (PortHandler *port, std::vector<UINT8_T> &id_list) = 0;

    virtual int Action          (PortHandler *port, UINT8_T id) = 0;
    virtual int Reboot          (PortHandler *port, UINT8_T id, UINT8_T *error = 0) = 0;
    virtual int FactoryReset    (PortHandler *port, UINT8_T id, UINT8_T option = 0, UINT8_T *error = 0) = 0;


    virtual int ReadTx          (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length) = 0;
    virtual int ReadRx          (PortHandler *port, UINT16_T length, UINT8_T *data, UINT8_T *error = 0) = 0;
    virtual int ReadTxRx        (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0) = 0;

    virtual int Read1ByteTx     (PortHandler *port, UINT8_T id, UINT16_T address) = 0;
    virtual int Read1ByteRx     (PortHandler *port, UINT8_T *data, UINT8_T *error = 0) = 0;
    virtual int Read1ByteTxRx   (PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T *data, UINT8_T *error = 0) = 0;

    virtual int Read2ByteTx     (PortHandler *port, UINT8_T id, UINT16_T address) = 0;
    virtual int Read2ByteRx     (PortHandler *port, UINT16_T *data, UINT8_T *error = 0) = 0;
    virtual int Read2ByteTxRx   (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T *data, UINT8_T *error = 0) = 0;

    virtual int Read4ByteTx     (PortHandler *port, UINT8_T id, UINT16_T address) = 0;
    virtual int Read4ByteRx     (PortHandler *port, UINT32_T *data, UINT8_T *error = 0) = 0;
    virtual int Read4ByteTxRx   (PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T *data, UINT8_T *error = 0) = 0;

    virtual int WriteTxOnly     (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data) = 0;
    virtual int WriteTxRx       (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0) = 0;

    virtual int Write1ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data) = 0;
    virtual int Write1ByteTxRx  (PortHandler *port, UINT8_T id, UINT16_T address, UINT8_T data, UINT8_T *error = 0) = 0;

    virtual int Write2ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data) = 0;
    virtual int Write2ByteTxRx  (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T data, UINT8_T *error = 0) = 0;

    virtual int Write4ByteTxOnly(PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data) = 0;
    virtual int Write4ByteTxRx  (PortHandler *port, UINT8_T id, UINT16_T address, UINT32_T data, UINT8_T *error = 0) = 0;

    virtual int RegWriteTxOnly  (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data) = 0;
    virtual int RegWriteTxRx    (PortHandler *port, UINT8_T id, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0) = 0;

    virtual int SyncReadTx      (PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length) = 0;
    // SyncReadRx   -> GroupSyncRead class
    // SyncReadTxRx -> GroupSyncRead class

    virtual int SyncWriteTxOnly (PortHandler *port, UINT16_T start_address, UINT16_T data_length, UINT8_T *param, UINT16_T param_length) = 0;

    virtual int BulkReadTx      (PortHandler *port, UINT8_T *param, UINT16_T param_length) = 0;
    // BulkReadRx   -> GroupBulkRead class
    // BulkReadTxRx -> GroupBulkRead class

    virtual int BulkWriteTxOnly (PortHandler *port, UINT8_T *param, UINT16_T param_length) = 0;
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_ */
