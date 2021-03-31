#ifndef PROTOCOLGOODWE_H
#define PROTOCOLGOODWE_H
#include "utils.h"

// defines contendo funcoes basicas
#define GOODWE_READ_FUNCTION            0x03
#define GOODWE_WRITE_FUNCTION           0x10

#define GOODWE_NOT_GOOD_FUNCTION        0x83
#define GOODWE_FAULTY_FUNCTION          0x90

// ADDRESS AND PROPERTY OF REGISTER

#define GOODWE_POWER_FACTOR             0x37
#define GOODWE_LOW_FEED_VOLT_HI_LO      0x00
#define GOODWE_LOW_FEED_VOLT_OFFSET_LO  0x01
#define GOODWE_LOW_FEED_VOLT            0x0000
#define GOODWE_FEEDING_PW_HI           0x02
#define GOODWE_FEEDING_PW_LO           0x33
#define GOODWE_FEEDING_PW              0x0233

#define GOODWE_INV_CONFIG_OFFSET_HI     0x00
#define GOODWE_INV_CONFIG_OFFSET_LO     0x05

#define GOODWE_HIGH_BYTE_00             0x00
#define GOODWE_RECONNECT_TIME           0x01
#define GOODWE_RECONNECT_TIME_WRITE     0x0001
#define GOODWE_HIGH_GRID_VOLTAGE        0x03
#define GOODWE_HIGH_GRID_VOLTAGE_W      0x0003
#define GOODWE_LOW_GRID_VOLTAGE         0x02
#define GOODWE_LOW_GRID_VOLTAGE_W       0x0002

#define GOODWE_HIGH_GRID_FREQUENCY      0x04
#define GOODWE_LOW_GRID_FREQUENCY       0x05
#define GOODWE_RTC_LO                   0x00
#define GOODWE_RTC_LENG                 0x06
#define GOODWE_RTC                      0x0010
#define GOODWE_RTC_OFFSET_HI            0x00
#define GOODWE_RTC_OFFSET_LO            0x03
#define GOODWE_RTC_YEAR_MONTH           0x10
#define GOODWE_RTC_DATE_HOUR            0x11
#define GOODWE_RTC_DATE_HOUR_LONG       0x0011
#define GOODWE_RTC_MINUTE_SECOND        0x12
#define GOODWE_RTC_MINUTE_SECOND_LONG   0x0012
// ADDRESS DE ESCRITA
#define GOODWE_RANGE_REAL_POWER_ADJUST_HI  0x00
#define GOODWE_RANGE_REAL_POWER_ADJUST_LO  0x01
#define GOODWE_RANGE_REAC_POWER_ADJUST_HI  0x01
#define GOODWE_RANGE_REAC_POWER_ADJUST_LO  0x01
#define GOODWE_RANGE_REAC_POWER            0x0101
// ADDRESS DE LEITURA
#define GOODWE_SERIAL_NUMBER_HI            0x02
#define GOODWE_SERIAL_NUMBER_LO            0x00
#define GOODWE_SERIAL_NUMBER               0x0200
#define GOODWE_MODEL_NAME_HI               0x02
#define GOODWE_MODEL_NAME_LO               0x10
#define GOODWE_MODEL_NAME                  0x0210
#define GOODWE_ALL_SET_OFFSET_HI           0x00
#define GOODWE_ALL_SET_OFFSET_LO           0x16
#define GOODWE_ABOUT_SERIAL                0x33
#define GOODWE_ABOUT_MODELO                0x35
#define GOODWE_ABOUT_MODELO_LENGTH         0x0a

#define GOODWE_ABOUT_SERIAL_OFFSET_HI      0x00
#define GOODWE_ABOUT_SERIAL_OFFSET_LO      0x08
#define GOODWE_ABOUT_MODELO_OFFSET_HI      0x00
#define GOODWE_ABOUT_MODELO_OFFSET_LO      0x05


#define GOODWE_ERROR_CODE                  0x0220
#define GOODWE_ERROR_CODE_HI               0x02
#define GOODWE_ERROR_CODE_LO               0x20
#define GOODWE_ETOTAL_HI                   0x02
#define GOODWE_ETOTAL_LO                   0x22
#define GOODWE_HTOTAL_HI                   0x02
#define GOODWE_HTOTAL_LO                   0x24
#define GOODWE_PV_VOLTAGE_1_HI             0x02
#define GOODWE_PV_VOLTAGE_1_LO             0x26
#define GOODWE_PV_VOLTAGE_2_HI             0x02
#define GOODWE_PV_VOLTAGE_2_LO             0x27
#define GOODWE_PV_CURRENT_1_HI             0x02
#define GOODWE_PV_CURRENT_1_LO             0x28
#define GOODWE_PV_CURRENT_2_HI             0x02
#define GOODWE_PV_CURRENT_2_LO             0x29
#define GOODWE_GRID_VOLTPHASE_1_HI         0x02
#define GOODWE_GRID_VOLTPHASE_1_LO         0x2A
#define GOODWE_GRID_VOLTPHASE_2_HI         0x02
#define GOODWE_GRID_VOLTPHASE_2_LO         0x2B
#define GOODWE_GRID_VOLTPHASE_3_HI         0x02
#define GOODWE_GRID_VOLTPHASE_3_LO         0x2C
#define GOODWE_GRID_CURPHASE_1_HI         0x02
#define GOODWE_GRID_CURPHASE_1_LO         0x2D
#define GOODWE_GRID_CURPHASE_2_HI         0x02
#define GOODWE_GRID_CURPHASE_2_LO         0x2E
#define GOODWE_GRID_CURPHASE_3_HI         0x02
#define GOODWE_GRID_CURPHASE_3_LO         0x2F
#define GOODWE_GRID_FREQPHASE_1_HI         0x02
#define GOODWE_GRID_FREQPHASE_1_LO         0x30
#define GOODWE_GRID_FREQPHASE_2_HI         0x02
#define GOODWE_GRID_FREQPHASE_2_LO         0x31
#define GOODWE_GRID_FREQPHASE_3_HI         0x02
#define GOODWE_GRID_FREQPHASE_3_LO         0x32
#define GOODWE_FEED_POWER_GRID_HI          0x02
#define GOODWE_FEED_POWER_GRID_LO          0x33
#define GOODWE_RUNNING_STATUS_HI           0x02
#define GOODWE_RUNNING_STATUS_LO           0x34
#define GOODWE_TEMPERATURE_HEAT_HI         0x02
#define GOODWE_TEMPERATURE_HEAT_LO         0x35
#define GOODWE_EDAY_HI         0x02
#define GOODWE_EDAY_LO         0x36

class ProtocolGoodWe
{
private:
    uint8_t bInverterAdd; // usado para query e para response
    uint8_t bFunctCode;  // usado para query e para response
    uint8_t bAddStartReg_hi; // usado para query
    uint8_t bAddStartReg_lo;  // usado para query
    uint8_t qnt_hi;
    uint8_t qnt_lo;
    uint8_t bamount_datas; // usado para response - quantidade de high e low
    uint8_t bamount_hi;  // usado para query -- para preset sera data preset high
    uint8_t bamount_lo; // usado para query -- para preset sera data preset low
    uint8_t bCRC16_hi;
    uint8_t bCRC16_lo;
    uint8_t bano;
    uint8_t bmes;
    uint8_t bdia;
    uint8_t bhora;
    uint8_t bmin;
    uint8_t bseg;
    uint16_t wCRC16;
    QByteArray mData;
    bool RTCenvio = false;
/* alternativa
    QByteArray mData_hi;
    QByteArray mData_lo;
*/
    uint16_t sCalStringSum(void);

public:
    ProtocolGoodWe();
    ProtocolGoodWe(uint8_t  bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi,uint8_t  bAddStartReg_lo, uint8_t bamount_hi,uint8_t bamount_lo);
    ProtocolGoodWe(uint8_t  bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi, uint8_t qnt_hi, uint8_t qnt_lo, uint8_t  bAddStartReg_lo, uint8_t bamount_datas,uint8_t bamount_hi,uint8_t bamount_lo);
    ProtocolGoodWe(uint8_t  bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi,uint8_t  bamount_datas);
    ProtocolGoodWe(QByteArray mRecvData);
    ProtocolGoodWe(uint8_t  bInverterAdd, uint8_t  bFunctCode, uint8_t  bAddStartReg_hi, uint8_t qnt_hi, uint8_t qnt_lo, uint8_t  bAddStartReg_lo, uint8_t bamount_datas,uint8_t bano,uint8_t bmes,uint8_t bdia,uint8_t bhora,uint8_t bminuto,uint8_t bseg);

    virtual ~ProtocolGoodWe();
    bool toBufferGoodWe(uint8_t abBuffer[]);
    bool checkAddresses(uint8_t bSrcAddr, uint8_t bDestAddr);
    uint16_t getPacketLength(void);
    uint8_t getFuncCode(void);
    uint8_t getHiAddr(void);
    uint8_t getLowAddr(void);
    QByteArray getData(void);
    bool checkAddressFunction(uint8_t addr, uint8_t funct);
    bool isValid(void);
    uint8_t getInverterAdd(void);
    void setInverterAdd( uint8_t addr);
};

#endif // PROTOCOLGOODWE_H
