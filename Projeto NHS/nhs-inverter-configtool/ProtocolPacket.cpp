#include "ProtocolPacket.h"

uint16_t ProtocolPacket::wCalcChecksum(void)
{
    uint16_t wBufSum = 0, wProtocolHeaderSum = 0;

    wProtocolHeaderSum = (wProtocolHeader & 0xFF) + ((wProtocolHeader & 0xFF00) >> 8);

    for (size_t i = 0; i < bDataLength; i++)
    {
        wBufSum += static_cast<uint8_t>(mData.at(static_cast<int>(i)));
    }
    wChecksum = wProtocolHeaderSum + bSrcAddr + bDestAddr + bCtrlCode + bFuncCode + bDataLength + wBufSum;

    return wChecksum;
}

ProtocolPacket::ProtocolPacket()
{
    wProtocolHeader = PACKET_PROTOCOL_HEADER;
    bSrcAddr = 0;
    bDestAddr = 0;
    bCtrlCode = 0;
    bFuncCode = 0;
    bDataLength = static_cast<uint8_t>(mData.length());
    wCalcChecksum();
    bProtocolLength = static_cast<uint8_t>(getPacketLength());
}

ProtocolPacket::ProtocolPacket(uint8_t bSrcAddr, uint8_t bDestAddr, uint8_t bCtrlCode, uint8_t bFuncCode, QByteArray mData) : bSrcAddr(bSrcAddr), bDestAddr(bDestAddr), bCtrlCode(bCtrlCode), bFuncCode(bFuncCode), mData(mData)
{
    wProtocolHeader = PACKET_PROTOCOL_HEADER;
    bDataLength = static_cast<uint8_t>(mData.length());
    wCalcChecksum();
    bProtocolLength = static_cast<uint8_t>(getPacketLength());
}

ProtocolPacket::ProtocolPacket(const char * pbRecvData)
{
    uint16_t wAux = 0;
    wProtocolHeader = static_cast<uint16_t>(*pbRecvData++);
    wAux = static_cast<uint16_t>(*pbRecvData++);
    wAux <<= 8;
    wProtocolHeader |= wAux;
    bSrcAddr = static_cast<uint8_t>(*pbRecvData++);
    bDestAddr = static_cast<uint8_t>(*pbRecvData++);
    bCtrlCode = static_cast<uint8_t>(*pbRecvData++);
    bFuncCode = static_cast<uint8_t>(*pbRecvData++);
    bDataLength = static_cast<uint8_t>(*pbRecvData++);

    mData.append(pbRecvData, bDataLength);
    pbRecvData += bDataLength;

    wChecksum = static_cast<uint16_t>(*pbRecvData++);
    wAux = static_cast<uint16_t>(*pbRecvData++);
    wAux <<= 8;
    wChecksum |= wAux;
}

ProtocolPacket::ProtocolPacket(QByteArray mRecvData)
{
    uint8_t wAux = 0;
    const char* pbRecvData = mRecvData.constData();

    bProtocolLength = 0x00;

    wProtocolHeader = static_cast<uint16_t>(*pbRecvData++);;
    wProtocolHeader <<= 8;
    wAux = static_cast<uint8_t>(*pbRecvData++);;
    wProtocolHeader |= wAux;
    bSrcAddr = static_cast<uint8_t>(*pbRecvData++);;
    bDestAddr = static_cast<uint8_t>(*pbRecvData++);;
    bCtrlCode = static_cast<uint8_t>(*pbRecvData++);;
    bFuncCode = static_cast<uint8_t>(*pbRecvData++);;
    bDataLength = static_cast<uint8_t>(*pbRecvData++);;

    mData.append(pbRecvData, bDataLength);
    pbRecvData += bDataLength;

    wChecksum = static_cast<uint16_t>(*pbRecvData++);;
    wChecksum <<= 8;
    wAux = static_cast<uint8_t>(*pbRecvData++);;
    wChecksum |= wAux;
}

ProtocolPacket::~ProtocolPacket()
{
}

bool ProtocolPacket::toBuffer(uint8_t abBuffer[], uint16_t wLength)
{
    uint8_t* pbBuffer = abBuffer;
    if (wLength < getPacketLength())
    {
        qDebug() << "Failed to parse packet into a buffer.\n";
        return false;
    }

    *pbBuffer++ = (wProtocolHeader & 0xFF00) >> 8;
    *pbBuffer++ = (wProtocolHeader & 0xFF);
    *pbBuffer++ = bSrcAddr;
    *pbBuffer++ = bDestAddr;
    *pbBuffer++ = bCtrlCode;
    *pbBuffer++ = bFuncCode;
    *pbBuffer++ = bDataLength;

    for (size_t i = 0; i < bDataLength; i++)
    {
        *pbBuffer++ = static_cast<uint8_t>(mData.at(static_cast<int>(i)));
    }

    *pbBuffer++ = (wChecksum & 0xFF00) >> 8;
    *pbBuffer++ = wChecksum & 0xFF;

    return true;
}

bool ProtocolPacket::isValid(void)
{
    return (wChecksum == wCalcChecksum());
}

bool ProtocolPacket::checkAddresses(uint8_t bSrcAddr, uint8_t bDestAddr)
{
    return ((this->bSrcAddr == bSrcAddr) && (this->bDestAddr == bDestAddr));
}

uint16_t ProtocolPacket::getPacketLength(void)
{
    return sizeof(wProtocolHeader) + sizeof(bSrcAddr) + sizeof(bDestAddr) + sizeof(bCtrlCode) + sizeof(bFuncCode) + sizeof(bDataLength) + bDataLength + sizeof(wChecksum);
}

uint8_t ProtocolPacket::getCtrlCode(void)
{
    return this->bCtrlCode;
}

uint8_t ProtocolPacket::getFuncCode(void)
{
    return this->bFuncCode;
}

QByteArray ProtocolPacket::getData(void)
{
    return this->mData;
}
