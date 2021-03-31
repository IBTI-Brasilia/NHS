#ifndef PROTOCOLGROWATT_H
#define PROTOCOLGROWATT_H
#include "utils.h"


// HOLDING REG ABout Info
#define GROWATT_AB_LO                   0x09
#define GROWATT_AB_HI                   0x00
#define GROWATT_OFFSET_AB_LO            0x15
#define GROWATT_OFFSET_AB_HI            0x00

// HOLDING RTC  Info
#define GROWATT_RTC_LO                   0x2D
#define GROWATT_RTC_HI                   0x00
#define GROWATT_OFFSET_RTC_LO            0x06
#define GROWATT_OFFSET_RTC_HI            0x00

// HOLDING MODELO INFO
#define GROWATT_MODELO_LO                0x1C
#define GROWATT_MODELO_HI                0x00
#define GROWATT_OFFSET_MODELO_LO         0x02
#define GROWATT_OFFSET_MODELO_HI         0x00


//Functions
#define GROWATT_READ_HOLDING 0x03
#define GROWATT_READ_INPUT 0x04
#define GROWATT_PRESET_SINGLE_REG 0x06
#define GROWATT_PRESET_MULT_REG 0x10

//Functions error
#define GROWATT_ERRO_READ_HOLDING 0x83
#define GROWATT_ERRO_READ_INPUT 0x84
#define GROWATT_ERRO_PRESET_SINGLE_REG 0x86
#define GROWATT_ERRO_PRESET_MULT_REG 0x90

//COMMANDS
#define GROWATT_READ_COMMAND 0x32
#define GROWATT_WRITE_COMMAND 0x33

//INPUT REGs PV1/ PV2
#define GROWATT_ALL_INFO_LO                   0x00
#define GROWATT_ALL_INFO_HI                   0x00
#define GROWATT_OFFSET_ALL_INFO_LO            0x29
#define GROWATT_OFFSET_ALL_INFO_HI            0x00



#define GROWATT_PV1_PV2_LO                   0x03
#define GROWATT_PV1_PV2_HI                   0x00
#define GROWATT_OFFSET_PV1_PV2_LO            0x08
#define GROWATT_OFFSET_PV1_PV2_HI            0x00

//INPUT REGs L1/L2/L3
#define GROWATT_L1_L2_L3_LO                   0x0D
#define GROWATT_L1_L2_L3_HI                   0x00
#define GROWATT_OFFSET_L1_L2_L3_LO            0x0D
#define GROWATT_OFFSET_L1_L2_L3_HI            0x00

//INPUT REGs STATUS INVERTER
#define GROWATT_STATUS_INVERTER_LO           0x00
#define GROWATT_STATUS_INVERTER_HI           0x00
#define GROWATT_OFFSET_STATUS_INVERTER_LO    0x01
#define GROWATT_OFFSET_STATUS_INVERTER_HI    0x00

//INPUT REGs TEMPERATURE
#define GROWATT_TEMP_LO                       0x20
#define GROWATT_TEMP_HI                       0x00
#define GROWATT_OFFSET_TEMP_LO                0x01
#define GROWATT_OFFSET_TEMP_HI                0x00

//INPUT REGs STATUS REDE
#define GROWATT_STATUS_REDE_LO                0x1C
#define GROWATT_STATUS_REDE_HI                0x00
#define GROWATT_OFFSET_STATUS_REDE_LO         0x04
#define GROWATT_OFFSET_STATUS_REDE_HI         0x00

//INPUT REGs STATUS ERROR
#define GROWATT_STATUS_ERROR_LO               0x28
#define GROWATT_STATUS_ERROR_HI               0x00
#define GROWATT_OFFSET_STATUS_ERROR_LO        0x01
#define GROWATT_OFFSET_STATUS_ERROR_HI        0x00


////////////////////////////////////////////////
//HOLDING REGs POWER FACTOR
#define GROWATT_POWER_FACTOR_LO              0x05
#define GROWATT_POWER_FACTOR_HI              0x00
#define GROWATT_OFFSET_POWER_FACTOR_LO       0x01
#define GROWATT_OFFSET_POWER_FACTOR_HI       0x00
//HOLDING REGs VAC low and High + TIME START + Fac low + Fac High
#define GROWATT_VAC_TIME_START_LO              0x12
#define GROWATT_VAC_TIME_START_HI              0x00
#define GROWATT_OFFSET_VAC_TIME_START_LO       0x05
#define GROWATT_OFFSET_VAC_TIME_START_HI       0x00

//WRITE HOLDING REGs TIME START + VAC + FAC hi and low

#define GROWATT_OFFSET_TIME_START_LO       0x01
#define GROWATT_OFFSET_TIME_START_HI       0x00

#define GROWATT_VAC_LOW_LO              0x13
#define GROWATT_VAC_LOW_HI              0x00
#define GROWATT_OFFSET_VAC_LOW_LO       0x01
#define GROWATT_OFFSET_VAC_LOW_HI       0x00

#define GROWATT_VAC_HIGH_LO              0x14
#define GROWATT_VAC_HIGH_HI              0x00
#define GROWATT_OFFSET_VAC_HIGH_LO       0x01
#define GROWATT_OFFSET_VAC_HIGH_HI       0x00

#define GROWATT_FAC_LOW_LO              0x15
#define GROWATT_FAC_LOW_HI              0x00
#define GROWATT_OFFSET_FAC_LOW_LO       0x01
#define GROWATT_OFFSET_FAC_LOW_HI       0x00

#define GROWATT_FAC_HIGH_LO              0x16
#define GROWATT_FAC_HIGH_HI              0x00
#define GROWATT_OFFSET_FAC_HIGH_LO       0x01
#define GROWATT_OFFSET_FAC_HIGH_HI       0x00

#define GROWATT_RECONNECT_TIME_LO        0x77
#define GROWATT_RECONNECT_TIME_HI        0x00
#define GROWATT_OFFSET_RECONNECT_TIME_LO 0x01
#define GROWATT_OFFSET_RECONNECT_TIME_HI 0x00
#define GROWATT_RECONNECT_TIME_QT        0x02

// WRITE HOLDING REGs RTC Year

#define GROWATT_RTC_YEAR_LO              0x2D
#define GROWATT_RTC_YEAR_HI              0x00
#define GROWATT_OFFSET_RTC_YEAR_LO       0x01
#define GROWATT_OFFSET_RTC_YEAR_HI       0x00

#define GROWATT_RTC_MONTH_LO              0x2E
#define GROWATT_RTC_MONTH_HI              0x00
#define GROWATT_OFFSET_RTC_MONTH_LO       0x01
#define GROWATT_OFFSET_RTC_MONTH_HI       0x00

#define GROWATT_RTC_DAY_LO              0x2F
#define GROWATT_RTC_DAY_HI              0x00
#define GROWATT_OFFSET_RTC_DAY_LO       0x01
#define GROWATT_OFFSET_RTC_DAY_HI       0x00

#define GROWATT_RTC_HOUR_LO              0x30
#define GROWATT_RTC_HOUR_HI              0x00
#define GROWATT_OFFSET_RTC_HOUR_LO       0x01
#define GROWATT_OFFSET_RTC_HOUR_HI       0x00

#define GROWATT_RTC_MINUTE_LO              0x31
#define GROWATT_RTC_MINUTE_HI              0x00
#define GROWATT_OFFSET_RTC_MINUTE_LO       0x01
#define GROWATT_OFFSET_RTC_MINUTE_HI       0x00

#define GROWATT_RTC_SEC_LO              0x32
#define GROWATT_RTC_SEC_HI              0x00
#define GROWATT_OFFSET_RTC_SEC_LO       0x01
#define GROWATT_OFFSET_RTC_SEC_HI       0x00

// WRITE POWER FACTOR
#define GROWATT_POWER_FACTOR_LO         0x05
#define GROWATT_POWER_FACTOR_HI         0x00
#define GROWATT_POWER_FACTOR_OFFSET_LO  0x01
#define GROWATT_POWER_FACTOR_OFFSET_HI  0x00
#define GROWATT_POWER_FACTOR_QT         0x02

// WRITE POWER FACTOR COMMAND
#define GROWATT_POWER_FACTOR_CMD_LO         0x02
#define GROWATT_POWER_FACTOR_CMD_HI         0x00
#define GROWATT_POWER_FACTOR_CMD_OFFSET_LO  0x01
#define GROWATT_POWER_FACTOR_CMD_OFFSET_HI  0x00

// WRITE POWER FACTOR MODEL
#define GROWATT_PF_MODEL_LO                 0x63
#define GROWATT_PF_MODE_HI                  0x00
#define GROWATT_PF_MODEL_OFFSET_LO          0x01
#define GROWATT_PF_MODEL_OFFSET_HI          0x00


// WRITE SPEC PASSWORD TYPE
#define GROWATT_PASSWORD_TYPE_LO            0x87
#define GROWATT_PASSWORD_TYPE_HI            0x00
#define GROWATT_PASSWORD_TYPE_VALUE         0x00
// WRITE SPEC PASSWORD 1,2,3
#define GROWATT_PASSWORD_LO            0x88
#define GROWATT_PASSWORD2_LO            0x89
#define GROWATT_PASSWORD3_LO            0x8A
#define GROWATT_PASSWORD_HI            0x00

// value X of password
#define GROWATT_PASSWORD_VALUE              0x58 // X = 0x58

#define GROWATT_TYPE_PASSWORD               0x00

#define GROWATT_BROADCAST_INVERSOR          0x00
#define GROWATT_WRITE_PF_MODEL_CMD          0x01


bool sCalcCRCReceived(QByteArray Array);

class ProtocolGrowatt
{
private:
    uint8_t bInverterAdd; // usado para query e para response
    uint8_t bFunctCode;  // usado para query e para response
    uint8_t bAddStartReg_hi; // usado para query
    uint8_t bAddStartReg_lo;  // usado para query
    uint8_t bamount_hi;  // usado para query -- para preset sera data preset high
    uint8_t bamount_lo; // usado para query -- para preset sera data preset low
    uint8_t bamount_datas; // usado para response - quantidade de high e low
    uint8_t bCRC16_hi;
    uint8_t bCRC16_lo;
    uint16_t wCRC16;
    QByteArray mData;
/* alternativa
    QByteArray mData_hi;
    QByteArray mData_lo;
*/
    uint16_t sCalStringSum(void);

public:
    ProtocolGrowatt();
    ProtocolGrowatt(uint8_t  bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi,uint8_t  bAddStartReg_lo, uint8_t bamount_hi,uint8_t bamount_lo);
    ProtocolGrowatt(uint8_t  bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi,uint8_t  bamount_datas);
    ProtocolGrowatt(QByteArray mRecvData);
    virtual ~ProtocolGrowatt();
    bool toBufferGrowatt(uint8_t abBuffer[]);
    bool checkAddresses(uint8_t bSrcAddr, uint8_t bDestAddr);
    uint16_t getPacketLength(void);
    uint8_t getFuncCode(void);
    uint8_t getLowAddr(void);
    uint8_t getHiAddr(void);
    QByteArray getData(void);
    bool checkAddressFunction(uint8_t addr, uint8_t funct);
    bool isValid(void);
    uint8_t getInverterAdd(void);
    void setInverterAdd( uint8_t addr);

};

#endif // PROTOCOLGROWATT_H
