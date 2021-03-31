#pragma once

#include <string.h>
#include <stdint.h>
#include <QDebug>

#define PACKET_PROTOCOL_HEADER  0xAA55UL

class ProtocolPacket
{
    private:
        uint8_t bProtocolLength;
        uint16_t wProtocolHeader;
        uint8_t bSrcAddr;
        uint8_t bDestAddr;
        uint8_t bCtrlCode;
        uint8_t bFuncCode;
        uint8_t bDataLength;
        QByteArray mData;
        uint16_t wChecksum;
        uint16_t wCalcChecksum(void);
    public:
        ProtocolPacket();
        ProtocolPacket(uint8_t bSrcAddr, uint8_t bDestAddr, uint8_t bCtrlCode, uint8_t bFuncCode, QByteArray mData);
        ProtocolPacket(const char * pbRecvData);
        ProtocolPacket(QByteArray mRecvData);
        virtual ~ProtocolPacket();
        bool toBuffer(uint8_t abBuffer[], uint16_t wLength);
        bool isValid(void);
        bool checkAddresses(uint8_t bSrcAddr, uint8_t bDestAddr);
        uint16_t getPacketLength(void);
        uint8_t getCtrlCode(void);
        uint8_t getFuncCode(void);
        QByteArray getData(void);
};

