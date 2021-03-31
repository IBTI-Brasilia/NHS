
#include "ProtocolGrowatt.h"

static const uint16_t wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };


ProtocolGrowatt::ProtocolGrowatt()
{
    bInverterAdd = INV_GROWATT_COM_ADD;
    bFunctCode = 0;
    bAddStartReg_hi = 0;
    bAddStartReg_lo = 0;
    bamount_hi = 0;
    bamount_lo = 0;
    bamount_datas = static_cast<uint8_t>(mData.length());
    sCalStringSum();
}
// Montando envio pacote para modos de leitura 0x03 0x04 e para preset single 0x06
ProtocolGrowatt::ProtocolGrowatt(uint8_t bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi,uint8_t  bAddStartReg_lo, uint8_t bamount_hi,uint8_t bamount_lo)
    :bInverterAdd(bInverterAdd), bFunctCode(bFunctCode), bAddStartReg_hi(bAddStartReg_hi), bAddStartReg_lo(bAddStartReg_lo), bamount_hi(bamount_hi), bamount_lo(bamount_lo)
{
//    bInverterAdd = INV_GROWATT_COM_ADD;
    bamount_datas = 0;
    sCalStringSum();
}
// Montando envio pacote para modos de leitura 0x03 0x04 e para preset single 0x06
ProtocolGrowatt::ProtocolGrowatt(uint8_t  bInverterAdd,uint8_t  bFunctCode ,uint8_t  bAddStartReg_hi,uint8_t  bamount_datas)
    :bInverterAdd(bInverterAdd), bFunctCode(bFunctCode), bAddStartReg_hi(bAddStartReg_hi), bamount_datas(bamount_datas)
{
//    bInverterAdd = INV_GROWATT_COM_ADD;
    bamount_datas = 0;
    sCalStringSum();
}

// Montando recieve <<RECV>> de pacote para modos de leitura 0x03 0x04 e para preset single 0x06
ProtocolGrowatt:: ProtocolGrowatt(QByteArray mRecvData) {

    uint8_t wAux = 0;
    const char* pbRecvData = mRecvData.constData();

    bInverterAdd = static_cast<uint8_t>(*pbRecvData++);
    bFunctCode = static_cast<uint8_t>(*pbRecvData++);
    bamount_datas = static_cast<uint8_t>(*pbRecvData++);
    qDebug()<<"Fora do if" << bamount_datas << " " << bFunctCode;
    if((bFunctCode == 0x86 || bFunctCode == 0x06) && bamount_datas == 0x00) {
           qDebug()<<"Dentro do if";
           bAddStartReg_hi = bamount_datas;
           bAddStartReg_lo = static_cast<uint8_t>(*pbRecvData++);
           bamount_hi = static_cast<uint8_t>(*pbRecvData++);
           bamount_lo = static_cast<uint8_t>(*pbRecvData++);

       } else {


           mData.append(pbRecvData, bamount_datas);
           pbRecvData += bamount_datas;
       }

    wCRC16 = static_cast<uint16_t>(*pbRecvData++);
    wAux = static_cast<uint8_t>(*pbRecvData++);
    wCRC16 <<= 8;
    wCRC16 |= wAux;

    bCRC16_lo = wCRC16 & 0xFF;
    bCRC16_hi = (wCRC16 >> 8) & 0xFF;

}
ProtocolGrowatt::~ProtocolGrowatt()
{


}


uint16_t ProtocolGrowatt::sCalStringSum(void)
{

    uint8_t i, aux1, aux2;
    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;
    // QByteArray * vecCRC;
    // vecCRC.append(bInverterAdd);
    // vecCRC.append(bFunctCode);

    uint8_t CrcData[6] = {bInverterAdd,bFunctCode,bAddStartReg_hi,bAddStartReg_lo,bamount_hi,bamount_lo};

    if (bamount_datas != 0x00) {
        // assume-se que eh recebimento de pacote logo 
        // bInverterAdd - bFunctCode - bAmount_datas  - Array[bData]
        uint8_t nDataRecieve[3] = {bInverterAdd,bFunctCode,bamount_datas};
        // vecCRC.append(amount_datas);
        // mData.append(mData, bamount_datas);

        // for(size_t cont = 0; cont <= vecCRC.lenght(); cont++ ) {
        //         nTemp = static_cast<uint8_t>(static_cast<uint8_t>(vecCRC.at(cont)) ^ wCRCWord);
        //         wCRCWord >>= 8;
        //         wCRCWord ^= wCRCTable[nTemp];
        // }
qDebug() << mData.toHex();
qDebug() << "Tamanho" << mData.length() << bamount_datas;


        for (i = 0; i < 3; i++) {
                nTemp = static_cast<uint8_t>(nDataRecieve[i] ^ wCRCWord);
                wCRCWord >>= 8;
                wCRCWord ^= wCRCTable[nTemp];
        }
        for(i = 0; i < bamount_datas; i++) {
                nTemp = static_cast<uint8_t>(static_cast<uint8_t>(mData.at(i)) ^ wCRCWord);
                wCRCWord >>= 8;
                wCRCWord ^= wCRCTable[nTemp];
        }
    } else {
        // assume-se que nao eh recebimento de pacote logo 
        // bInverterAdd - bFunctCode - bAdd_hi - bAddlo - bqnthi - bqntlo
        //////////////////////////////////////////////
        // vecCRC.append(bAddStartReg_hi);
        // vecCRC.append(bAddStartReg_lo);
        // vecCRC.append(bamount_hi);
        // vecCRC.append(bamount_lo);
        // for(size_t cont = 0; cont <= vecCRC.lenght(); cont++ ) {
        //         nTemp = static_cast<uint8_t>(static_cast<uint8_t>(vecCRC.at(cont)) ^ wCRCWord);
        //         wCRCWord >>= 8;
        //         wCRCWord ^= wCRCTable[nTemp];
        // }

            for (i = 0; i <= 5; i++) {
                nTemp = static_cast<uint8_t>(CrcData[i] ^ wCRCWord);
                wCRCWord >>= 8;
                wCRCWord ^= wCRCTable[nTemp];
            }
    }
        aux1 = wCRCWord & 0xFF;
        aux2 = (wCRCWord >> 8) & 0xFF;
        bCRC16_hi = aux1;
        bCRC16_lo = aux2;

    uint16_t saidaCRC = static_cast<uint16_t>(aux1 << 8)|aux2;
    //wCRC16 = saidaCRC;
    return saidaCRC;
}

bool ProtocolGrowatt::toBufferGrowatt(uint8_t abBuffer[]) {

    uint8_t* pbBuffer = abBuffer;

    *pbBuffer++ = (bInverterAdd);
    *pbBuffer++ = bFunctCode;
    *pbBuffer++ = bAddStartReg_hi;
    *pbBuffer++ = bAddStartReg_lo;
    *pbBuffer++ = bamount_hi;
    *pbBuffer++ = bamount_lo;
    *pbBuffer++ = bCRC16_hi;
    *pbBuffer++ = bCRC16_lo;

    return true;
}

uint16_t ProtocolGrowatt::getPacketLength(void) {
    size_t aux;
    aux = sizeof(bInverterAdd) + sizeof(bFunctCode) + sizeof(bCRC16_lo) + sizeof(bCRC16_hi);

       if (bamount_datas != 0x00) {
        return static_cast<uint16_t>(aux + sizeof(bamount_datas) + static_cast<size_t>(bamount_datas));
    }
       else {
        return static_cast<uint16_t>(aux + sizeof(bAddStartReg_hi) + sizeof(bAddStartReg_lo) + sizeof(bamount_hi) + sizeof(bamount_lo));
            }
    }
 

uint8_t ProtocolGrowatt::getFuncCode(void) {
    return this->bFunctCode;
}

uint8_t ProtocolGrowatt::getHiAddr(void) {
    return this->bAddStartReg_hi;
}
uint8_t ProtocolGrowatt::getLowAddr(void) {
    return this->bAddStartReg_lo;
}

QByteArray ProtocolGrowatt::getData(void) {
    return this->mData;
}

uint8_t ProtocolGrowatt::getInverterAdd(void) {
    return this->bInverterAdd;
}

void ProtocolGrowatt::setInverterAdd( uint8_t addr) {
    this->bInverterAdd = addr;
}

bool ProtocolGrowatt::checkAddressFunction(uint8_t addr, uint8_t funct){
#if 1
    qDebug() << this->bInverterAdd << "==" << addr << "&&" << this->bFunctCode << "==" << funct;
#endif
//    return (this->bInverterAdd == addr && this->bFunctCode == funct);
    return (this->bInverterAdd == addr);
}

bool ProtocolGrowatt::isValid(void)
{
#if 1
    qDebug() << "Comparando CRC"<< wCRC16 << hex << this->wCRC16 << "==" << hex << this->sCalStringSum();
#endif
   return (this->wCRC16 == this->sCalStringSum());

}
