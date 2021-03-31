#pragma once
/**********************************************************************
 * Protocol for solar inverter family
 * ********************************************************************/
/**********************************************************************
 * Includes
 * ********************************************************************/
// Types Includes
#include <stdint.h>
#include <string>
#include <QtSerialPort>
#include <qapplication.h>
/**********************************************************************
 * Defines
 * ********************************************************************/

enum{
    CONNECTED,
    PORT_ERROR,
    INV_ERROR
};

// Control Code
#define INV_CNTL_CODE_REGISTER  0x00
#define INV_CNTL_CODE_READ      0x01
#define INV_CNTL_CODE_EXECUTE   0x02
#define INV_CNTL_CODE_EXECUTE_T 0x03

// Function code to register control code
// Application to inverter functions
#define INV_REG_FUNC_CODE_OFFLINE_QUERY             0x00
#define INV_REG_FUNC_CODE_ALLOCATE_REG_ADDRESS      0x01
#define INV_REG_FUNC_CODE_REMOVE_REGISTER           0x02

// Inverter to application functions
#define INV_REG_FUNC_CODE_REGISTER_REQUEST          0x80
#define INV_REG_FUNC_CODE_ADDRESS_CONFIRM           0x81
#define INV_REG_FUNC_CODE_REMOVE_CONFIRM            0x82

// Function code to read control code
// Application to inverter functions
#define INV_RD_FUNC_CODE_QUERY_READ_DATA_LIST       0x00
#define INV_RD_FUNC_CODE_QUERY_RUNNING_INFO         0x01
#define INV_RD_FUNC_CODE_QUERY_ID_INFO              0x02
#define INV_RD_FUNC_CODE_QUERY_SETTING_INFO         0x03
#define INV_RD_FUNC_CODE_QUERY_ERROR_MESSAGE        0x04
#define INV_RD_FUNC_CODE_READ_STORAGE_RUNNING_INFO  0x06
#define INV_RD_FUNC_CODE_READ_RTC_TIME_VALUE        0x07
#define INV_RD_FUNC_CODE_READ_ES_SETTING_INFO       0x09

// Inverter to application functions
#define INV_RD_FUNC_CODE_RESPONSE_READ_DATA_LIST        0x80
#define INV_RD_FUNC_CODE_RESPONSE_RUNNING_INFO          0x81
#define INV_RD_FUNC_CODE_RESPONSE_ID_INFO               0x82
#define INV_RD_FUNC_CODE_RESPONSE_SETTING_INFO          0x83
#define INV_RD_FUNC_CODE_RESPONSE_ERROR_MESSAGE         0x84
#define INV_RD_FUNC_CODE_RESPONSE_STORAGE_RUNNING_INFO  0x86
#define INV_RD_FUNC_CODE_RESPONSE_RTC_TIME_VALUE        0x87
#define INV_RD_FUNC_CODE_RESPONSE_READ_ES_SETTING_INFO  0x89


// Function code to execute control code
// Application to inverter functions
#define INV_EXC_FUNC_CODE_SET_SAFETY_COUNTRY            0x01
#define INV_EXC_FUNC_CODE_SET_RTC_TIME                  0x02
#define INV_EXC_FUNC_CODE_SET_SETTING                   0x06
#define INV_EXC_FUNC_CODE_SET_GRID_VOLTAGE_CALIB        0x10    //!< ATS only
#define INV_EXC_FUNC_CODE_SET_PWR_ADJUST_RATING         0x12    //!< ATS only
#define INV_EXC_FUNC_CODE_SET_MAXIMUM_PAC_ADJUST        0x13    //!< ATS only
#define INV_EXC_FUNC_CODE_SET_PV_VOLTAGE_CALIB          0x14    //!< ATS only
#define INV_EXC_FUNC_CODE_SET_WATT_RATIO                0x15    //!< ATS only
#define INV_EXC_FUNC_CODE_GRID_CURRENT_CALIB            0x16    //!< ATS only
#define INV_EXC_FUNC_CODE_RECONNECT                     0x1D
#define INV_EXC_FUNC_CODE_ADJUST_REAL_POWER             0x1E
#define INV_EXC_FUNC_CODE_ADJUST_REACTIVE_PWR           0x1F    // used by power factor, see the table 3-22;
#define INV_EXC_FUNC_CODE_START_STD_PF_CURVE            0x20
#define INV_EXC_FUNC_CODE_STOP_STD_PF_CURVE             0x21
#define INV_EXC_FUNC_CODE_SET_STORAGE_WORK_MODE         0x26
#define INV_EXC_FUNC_CODE_SET_RELAY_DRIVER              0x27
#define INV_EXC_FUNC_CODE_FUNCTION_SWITCH_CMD           0x28
#define INV_EXC_FUNC_CODE_SCI_CHANNEL_SWITCH            0x2A
#define INV_EXC_FUNC_CODE_SET_CHARGE_TIME               0x2C
#define INV_EXC_FUNC_CODE_SET_DISCHARGE_TIME            0x2D
#define INV_EXC_FUNC_CODE_SET_LEAD_ACID_BATTERY_INFO    0x50
#define INV_EXC_FUNC_CODE_SET_CHARGE_PARAMENT           0x51
#define INV_EXC_FUNC_CODE_SET_DISCHARGE_PARAMENT        0x52
#define INV_EXC_FUNC_CODE_FEEDING_PWR2GRID_FUNCTION     0x53
#define INV_EXC_FUNC_CODE_SIMULATE_METER_POWER          0x54
#define INV_EXC_FUNC_CODE_SET_BATTERY_SAFETY_CURVE      0x55
#define INV_EXC_FUNC_CODE_SET_CHARGING_VOLT_CURR_TIME   0x56
#define INV_EXC_FUNC_CODE_SET_EQUALIZATION_VOLT_TIME    0x57
#define INV_EXC_FUNC_CODE_SET_POWER_MODE                0x58
#define INV_EXC_FUNC_CODE_CONTROL_ES_WIFI_LED           0x5A
#define INV_EXC_FUNC_CODE_GRID_WAVE_QUALITY_DET_LEVEL   0x5B

// Inverter to application functions 
#define INV_EXC_FUNC_CODE_RESPONSE_SET_SAFETY_COUNTRY            0x81
#define INV_EXC_FUNC_CODE_RESPONSE_SET_RTC_TIME                  0x82
#define INV_EXC_FUNC_CODE_RESPONSE_SET_SETTING                   0x86
#define INV_EXC_FUNC_CODE_RESPONSE_SET_GRID_VOLTAGE_CALIB        0x90    //!< ATS only
#define INV_EXC_FUNC_CODE_RESPONSE_SET_PWR_ADJUST_RATING         0x92    //!< ATS only
#define INV_EXC_FUNC_CODE_RESPONSE_SET_MAXIMUM_PAC_ADJUST        0x93    //!< ATS only
#define INV_EXC_FUNC_CODE_RESPONSE_SET_PV_VOLTAGE_CALIB          0x94    //!< ATS only
#define INV_EXC_FUNC_CODE_RESPONSE_SET_WATT_RATIO                0x95    //!< ATS only
#define INV_EXC_FUNC_CODE_RESPONSE_GRID_CURRENT_CALIB            0x96    //!< ATS only
#define INV_EXC_FUNC_CODE_RESPONSE_RECONNECT                     0x9D
#define INV_EXC_FUNC_CODE_RESPONSE_ADJUST_REAL_POWER             0x9E
#define INV_EXC_FUNC_CODE_RESPONSE_ADJUST_REACTIVE_PWR           0x9F
#define INV_EXC_FUNC_CODE_RESPONSE_START_STD_PF_CURVE            0xA0
#define INV_EXC_FUNC_CODE_RESPONSE_STOP_STD_PF_CURVE             0xA1
#define INV_EXC_FUNC_CODE_RESPONSE_SET_STORAGE_WORK_MODE         0xA6
#define INV_EXC_FUNC_CODE_RESPONSE_SET_RELAY_DRIVER              0xA7
#define INV_EXC_FUNC_CODE_RESPONSE_FUNCTION_SWITCH_CMD           0xA8
#define INV_EXC_FUNC_CODE_RESPONSE_SCI_CHANNEL_SWITCH            0xAA
#define INV_EXC_FUNC_CODE_RESPONSE_SET_CHARGE_TIME               0xAC
#define INV_EXC_FUNC_CODE_RESPONSE_SET_DISCHARGE_TIME            0xAD
#define INV_EXC_FUNC_CODE_RESPONSE_SET_LEAD_ACID_BATTERY_INFO    0xD0
#define INV_EXC_FUNC_CODE_RESPONSE_SET_CHARGE_PARAMENT           0xD1
#define INV_EXC_FUNC_CODE_RESPONSE_SET_DISCHARGE_PARAMENT        0xD2
#define INV_EXC_FUNC_CODE_RESPONSE_FEEDING_PWR2GRID_FUNCTION     0xD3
#define INV_EXC_FUNC_CODE_RESPONSE_SIMULATE_METER_POWER          0xD4
#define INV_EXC_FUNC_CODE_RESPONSE_SET_BATTERY_SAFETY_CURVE      0xD5
#define INV_EXC_FUNC_CODE_RESPONSE_SET_CHARGING_VOLT_CURR_TIME   0xD6
#define INV_EXC_FUNC_CODE_RESPONSE_SET_EQUALIZATION_VOLT_TIME    0xD7
#define INV_EXC_FUNC_CODE_RESPONSE_SET_POWER_MODE                0xD8
#define INV_EXC_FUNC_CODE_RESPONSE_CONTROL_ES_WIFI_LED           0xDA
#define INV_EXC_FUNC_CODE_RESPONSE_GRID_WAVE_QUALITY_DET_LEVEL   0xDB


// Utils
#define INV_EXC_RESPONSE_CHECKSUM_FAIL      0x00
#define INV_EXC_RESPONSE_EXECUTE_SUCCESS    0x06
#define INV_EXC_RESPONSE_EXECUTE_FAIL       0x15

#define INV_EXC_ES_INV_WORK_MODE_WAIT_MODE      0x01
#define INV_EXC_ES_INV_WORK_MODE_ONLINE_MODE    0x02
#define INV_EXC_ES_INV_WORK_MODE_OFFLINE_MODE   0x04
#define INV_EXC_ES_INV_WORK_MODE_FAULT_MODE     0x10
#define INV_EXC_ES_INV_WORK_MODE_VF_START_MODE  0x20

#define INV_EXC_PV_MODE_NO_PV    0x00
#define INV_EXC_PV_MODE_STANDBY  0x01
#define INV_EXC_PV_MODE_WORK     0x02

#define INV_EXC_BATTERY_MODE_NO_BATTERY     0x00
#define INV_EXC_BATTERY_MODE_STANDBY        0x01
#define INV_EXC_BATTERY_MODE_DISCHARGING    0x02
#define INV_EXC_BATTERY_MODE_CHARGING       0x03

#define INV_EXC_GRID_MODE_LOSS     0x00
#define INV_EXC_GRID_MODE_OK       0x01
#define INV_EXC_GRID_MODE_FAULT    0x02

#define INV_EXC_LOAD_MODE_ON    0x00
#define INV_EXC_LOAD_MODE_OFF   0x01

#define INV_RUNNING_INFO_PV1_VOLTAGE_OFFSET         0x00
#define INV_RUNNING_INFO_PV2_VOLTAGE_OFFSET         0x02
#define INV_RUNNING_INFO_PV1_CURRENT_OFFSET         0x04
#define INV_RUNNING_INFO_PV2_CURRENT_OFFSET         0x06
#define INV_RUNNING_INFO_PHASE_L1_VOLTAGE_OFFSET    0x08
#define INV_RUNNING_INFO_PHASE_L2_VOLTAGE_OFFSET    0x0A
#define INV_RUNNING_INFO_PHASE_L3_VOLTAGE_OFFSET    0x0C
#define INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET    0x0E
#define INV_RUNNING_INFO_PHASE_L2_CURRENT_OFFSET    0x10
#define INV_RUNNING_INFO_PHASE_L3_CURRENT_OFFSET    0x12
#define INV_RUNNING_INFO_PHASE_L1_FREQUENCY_OFFSET  0x14
#define INV_RUNNING_INFO_PHASE_L2_FREQUENCY_OFFSET  0x16
#define INV_RUNNING_INFO_PHASE_L3_FREQUENCY_OFFSET  0x18
#define INV_RUNNING_INFO_LB_FEEDING_POWER_OFFSET    0x1A
#define INV_RUNNING_INFO_WORK_MODE_OFFSET           0x1C
#define INV_RUNNING_INFO_INTERNAL_TEMP_OFFSET       0x1E
#define INV_RUNNING_INFO_ERROR_MESSAGE_H_OFFSET     0x20
#define INV_RUNNING_INFO_ERROR_MESSAGE_L_OFFSET     0x22
#define INV_RUNNING_INFO_TOTAL_ENERGY2GRID_H_OFFSET 0x24
#define INV_RUNNING_INFO_TOTAL_ENERGY2GRID_L_OFFSET 0x26
#define INV_RUNNING_INFO_TOTAL_FEED_HOURS_H_OFFSET  0x28
#define INV_RUNNING_INFO_TOTAL_FEED_HOURS_L_OFFSET  0x2A

#define GWE_INV_RTC_TIME_INFO_YEAR_OFFSET           0
#define GWE_INV_RTC_TIME_INFO_MONTH_OFFSET          1
#define GWE_INV_RTC_TIME_INFO_DAY_OFFSET            2
#define GWE_INV_RTC_TIME_INFO_HOUR_OFFSET           3
#define GWE_INV_RTC_TIME_INFO_MINUTE_OFFSET         4
#define GWE_INV_RTC_TIME_INFO_SECOND_OFFSET         5

#define GWT_INV_RTC_TIME_INFO_YEAR_OFFSET           1
#define GWT_INV_RTC_TIME_INFO_MONTH_OFFSET          3
#define GWT_INV_RTC_TIME_INFO_DAY_OFFSET            5
#define GWT_INV_RTC_TIME_INFO_HOUR_OFFSET           7
#define GWT_INV_RTC_TIME_INFO_MINUTE_OFFSET         9
#define GWT_INV_RTC_TIME_INFO_SECOND_OFFSET         11

#define GWE_INV_ID_INFO_FW_VERSION_OFFSET           0
#define GWE_INV_ID_INFO_MODEL_NAME_OFFSET           5
#define GWE_INV_ID_INFO_MANUFACTURER_OFFSET         15
#define GWE_INV_ID_INFO_SERIAL_NUMBER_OFFSET        31
#define GWE_INV_ID_INFO_PV_VOLTAGE_OFFSET           47
#define GWE_INV_ID_INFO_INTERNAL_VERSION_OFFSET     51
#define GWE_INV_ID_INFO_SAFETY_COUNTRY_CODE_OFFSET  63

#define GWE_INV_ID_INFO_FW_VERSION_DATA_LENGTH           5
#define GWE_INV_ID_INFO_MODEL_NAME_DATA_LENGTH           10
#define GWE_INV_ID_INFO_MANUFACTURER_DATA_LENGTH         16
#define GWE_INV_ID_INFO_SERIAL_NUMBER_DATA_LENGTH        16
#define GWE_INV_ID_INFO_PV_VOLTAGE_DATA_LENGTH           4
#define GWE_INV_ID_INFO_INTERNAL_VERSION_DATA_LENGTH     12
#define GWE_INV_ID_INFO_SAFETY_COUNTRY_CODE_DATA_LENGTH  1

#define GWE_INV_POWER_FACTOR_RESPONSE_ES_OFFSET         42

#define GWT_INV_ID_INFO_FW_VERSION_OFFSET           0
#define GWT_INV_ID_INFO_SERIAL_NUMBER_OFFSET        28
#define GWT_INV_ID_INFO_MODEL_NAME_OFFSET           38

#define GWT_INV_ID_INFO_FW_VERSION_DATA_LENGTH      6
#define GWT_INV_ID_INFO_SERIAL_NUMBER_DATA_LENGTH   10
#define GWT_INV_ID_INFO_MODEL_NAME_DATA_LENGTH      4

#define INV_SETTING_INFO_PV_STARTUP_VOLTAGE_OFFSET  0
#define INV_SETTING_INFO_TIME2CONNECT_GRID_OFFSET   2
#define INV_SETTING_INFO_MIN_GRID_VOLTAGE_OFFSET    4
#define INV_SETTING_INFO_MAX_GRID_VOLTAGE_OFFSET    6
#define INV_SETTING_INFO_MIN_GRID_FREQUENCY_OFFSET  8
#define INV_SETTING_INFO_MAX_GRID_FREQUENCY_OFFSET  10


#define GWT_INV_STATUS_OFFSET                       0
#define GWT_INV_PV1_VOLT_OFFSET                     6
#define GWT_INV_PV1_CURR_OFFSET                     8
#define GWT_INV_PV2_VOLT_OFFSET                     14
#define GWT_INV_PV2_CURR_OFFSET                     16
#define GWT_INV_GRID_FREQUENCY_OFFSET               26
#define GWT_INV_VAC1_OFFSET                         28
#define GWT_INV_IAC1_OFFSET                         30
#define GWT_INV_VAC2_OFFSET                         36
#define GWT_INV_IAC2_OFFSET                         38
#define GWT_INV_VAC3_OFFSET                         44
#define GWT_INV_IAC3_OFFSET                         46
#define GWT_INV_ENE_TODAY_HI_OFFSET                 52
#define GWT_INV_ENE_TODAY_LO_OFFSET                 54
#define GWT_INV_ENE_TOTAL_HI_OFFSET                 56
#define GWT_INV_ENE_TOTAL_LO_OFFSET                 58
#define GWT_INV_TIME_TOTAL_HI_OFFSET                60
#define GWT_INV_TIME_TOTAL_LO_OFFSET                62
#define GWT_INV_TEMPERATURE_OFFSET                  64
#define GWT_INV_FAULT_CODE_OFFSET                   80


#define INV_SETTING_PV_START_VOLTAGE_VALUE          1100U
#define INV_SETTING_MIN_GRID_FREQUENCY_VALUE        5747U
#define INV_SETTING_MAX_GRID_FREQUENCY_VALUE        6197U
#define INV_SETTING_208V_MIN_GRID_VOLTAGE_VALUE     1664U
#define INV_SETTING_208V_MAX_GRID_VOLTAGE_VALUE     2288U
#define INV_SETTING_220V_MIN_GRID_VOLTAGE_VALUE     1760U
#define INV_SETTING_220V_MAX_GRID_VOLTAGE_VALUE     2420U
#define INV_SETTING_230V_MIN_GRID_VOLTAGE_VALUE     1840U
#define INV_SETTING_230V_MAX_GRID_VOLTAGE_VALUE     2530U
#define INV_SETTING_240V_MIN_GRID_VOLTAGE_VALUE     1920U
#define INV_SETTING_240V_MAX_GRID_VOLTAGE_VALUE     2640U
#define INV_SETTING_254V_MIN_GRID_VOLTAGE_VALUE     2032U
#define INV_SETTING_254V_MAX_GRID_VOLTAGE_VALUE     2794U


#define GWT_SETTING_208V_MIN_GRID_VOLTAGE_VALUE     1664U
#define GWT_SETTING_208V_MAX_GRID_VOLTAGE_VALUE     2288U
#define GWT_SETTING_220V_MIN_GRID_VOLTAGE_VALUE     1760U
#define GWT_SETTING_220V_MAX_GRID_VOLTAGE_VALUE     2420U
#define GWT_SETTING_230V_MIN_GRID_VOLTAGE_VALUE     1840U
#define GWT_SETTING_230V_MAX_GRID_VOLTAGE_VALUE     2530U
#define GWT_SETTING_240V_MIN_GRID_VOLTAGE_VALUE     1920U
#define GWT_SETTING_240V_MAX_GRID_VOLTAGE_VALUE     2640U
#define GWT_SETTING_254V_MIN_GRID_VOLTAGE_VALUE     2032U
#define GWT_SETTING_254V_MAX_GRID_VOLTAGE_VALUE     2794U

#define INV_MASTER_ADDRESS			0xB0
#define INV_INITIAL_SLAVE_ADDRESS	0x7F
#define INV_SERIAL_NUMBER_LENGTH	16
#define INV_COMM_WRITE_ERROR        -1
#define INV_GROWATT_COM_ADD         0x00 // depois mudar para broadcast e testar em tese nao mudara nada


#define INV_CONFIG_OUTPUT_VOLTAGE_RANGE_MIN     20
#define INV_CONFIG_OUTPUT_VOLTAGE_RANGE_MAX     300

#define INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL  0x00
#define INV_EXEC_ACKNOWLEDGEMENT_ACK            0x06
#define INV_EXEC_ACKNOWLEDGEMENT_NACK           0x15

#define INV_COMM_INVERTER_CONNECTED_TEXT        "Inversor conectado"
#define INV_COMM_INVERTER_DISCONNECTED_TEXT     "Inversor desconectado"
#define INV_COMM_INVERTER_CONNECT_ERROR_TEXT    "Erro ao conectar inversor"
#define INV_COMM_INVERTER_NOT_FOUND_TEXT        "Inversor não encontrado"
#define PORT_NOT_FOUND_TEXT                     "Porta não encontrada"

#define INV_CFG_QMSGBOX_INVALID_RANGE_TITLE     "Valor inválido"
#define INV_CFG_QMSGBOX_INVALID_RANGE_TEXT      "Por favor, insira um valor entre 20 e 300"
#define INV_CFG_QMSPOWFACTOR_INVALID_RANGE_TEXT "Por favor, insira um valor entre 0.0000 e 1.0000"

#define INV_CFG_QMSGBOX_NEW_SETTINGS_TITLE      "Configuração"
#define INV_CFG_QMSGBOX_NEW_SETTINGS_TEXT       "Configuração alterada com sucesso"
#define INV_CFG_QMSGBOX_NEW_SETTINGS_ERROR_TEXT "Erro ao alterar a configuração \n Por favor, tente novamente"
#define INV_CFG_QMSGBOX_NEW_OPEN_SERIAL_PORT    "Porta configurada com sucesso"

#define INV_CFG_QMSGBOX_NEW_RTC_TIME_TITLE      "Sincronização RTC"
#define INV_CFG_QMSGBOX_NEW_RTC_TIME_TEXT       "RTC sincronizado com sucesso"
#define INV_CFG_QMSGBOX_NEW_RTC_TIME_ERROR_TEXT "Erro ao sincronizar o RTC"

#define INV_CFG_QMSGBOX_NEW_POW_FAC_TITLE      "Fator de Potência"
#define INV_CFG_QMSGBOX_NEW_POW_FAC_TEXT       "Fator de potência alterado com sucesso"
#define INV_CFG_QMSGBOX_NEW_POW_FAC_ERROR_TEXT "Erro ao alterar o fator de potência"

#define INV_CFG_MSG_GWE_GWT_TITLE               "Inversor Conectado"
#define INV_CFG_MSG_GWE_GWT_GWT_DECIDE          "Inversor GROWATT Conectado"
#define INV_CFG_MSG_GWE_GWT_GWE_DECIDE          "Inversor GOODWE Conectado"
#define INV_CFG_MSG_GWE_GWT_ERROR_DECIDE        "Erro ao identificar inversor"

#define INV_WRITE_OR_READ_ERRO_GOODWE           "Erro na mensagem de retorno do goodwe"
#define INV_UI_INFO_TAB_INDEX       0
#define INV_UI_CONFIG_TAB_INDEX     1
#define INV_UI_ABOUT_TAB_INDEX      2


/**********************************************************************
 * Typedefs
 * ********************************************************************/
typedef struct {
    uint8_t bSerialNumber[INV_SERIAL_NUMBER_LENGTH];  // Holds the inverter serial number (16 bytes). Filled by register request message.
    uint8_t bInvAddress;        // Holds the inverter allocated address
} sAllocateAddressData;

typedef enum {
    INV_GOODWE     = 0U,
    INV_GROWATT    = 1U,
    INV_GOODWE_MOD = 2U,
    INV_DEC_ERRO   =  3U,

} eDecideInverter;

typedef enum {
    INV_WORK_MODE_WAIT      = 0x0000,
    INV_WORK_MODE_NORMAL    = 0x0001,
    INV_WORK_MODE_FAULT     = 0x0002,
} eInverterWorkMode;

typedef struct {
    float fPV1Voltage;
    float fPV2Voltage;
    float fPV1Current;
    float fPV2Current;
    float fPhaseL1Voltage;
    float fPhaseL2Voltage;
    float fPhaseL3Voltage;
    float fPhaseL1Current;
    float fPhaseL2Current;
    float fPhaseL3Current;
    float fPhaseL1Frequency;
    float fPhaseL2Frequency;
    float fPhaseL3Frequency;
    uint16_t wLowByteFeedingPW;
    uint8_t bPowerFactor;
    eInverterWorkMode eInvWorkMode;
    float fInternalTemp;
    uint32_t dErrorMessage;
    float fTotalEnergy2Grid;
    uint32_t dTotalFeedHours;
    uint8_t bSafetyCountryCode;
} sRunningInfoData;
Q_DECLARE_METATYPE(sRunningInfoData);

typedef enum {
    GWT_WORK_MODE_WAIT      = 0x0000,
    GWT_WORK_MODE_NORMAL    = 0x0001,
    GWT_WORK_MODE_FAULT     = 0x0003,
} eInvGWTWorkMode_tDef;

typedef struct {
    float fPV1Voltage;
    float fPV2Voltage;
    float fPV1Current;
    float fPV2Current;
    float fPhaseL1Voltage;
    float fPhaseL2Voltage;
    float fPhaseL3Voltage;
    float fPhaseL1Current;
    float fPhaseL2Current;
    float fPhaseL3Current;
    float fPhaseL1Frequency;
    float fPhaseL2Frequency;
    float fPhaseL3Frequency;
    uint16_t wLowByteFeedingPW;
    uint8_t bPowerFactor;
    eInvGWTWorkMode_tDef eInvGWTWorkMode;
    float fInternalTemp;
    uint16_t dErrorMessage;
    float fTotalEnergy2Grid;
    float dTotalFeedHours;
//    uint8_t bSafetyCountryCode;
} sGWTRunInfoData;
Q_DECLARE_METATYPE(sGWTRunInfoData);

typedef struct {
    uint8_t abFWVersion[GWE_INV_ID_INFO_FW_VERSION_DATA_LENGTH + 1] ;
    uint8_t abManufacturer[GWE_INV_ID_INFO_MANUFACTURER_DATA_LENGTH+1]= {0};
    uint8_t abSerialNumber[GWE_INV_ID_INFO_SERIAL_NUMBER_DATA_LENGTH+1]= {0};
    uint8_t abNomPVVoltage[GWE_INV_ID_INFO_PV_VOLTAGE_DATA_LENGTH+1]= {0};
    uint8_t abInternalVersion[GWE_INV_ID_INFO_INTERNAL_VERSION_DATA_LENGTH+1]= {0};
    uint8_t abModelName[GWE_INV_ID_INFO_MODEL_NAME_DATA_LENGTH+1]= {0};
} sIDInfoDataGWE;

typedef struct {
    uint8_t abFWVersion[GWT_INV_ID_INFO_FW_VERSION_DATA_LENGTH];
    uint8_t abModelName[GWT_INV_ID_INFO_MODEL_NAME_DATA_LENGTH];
    uint8_t abSerialNumber[GWT_INV_ID_INFO_SERIAL_NUMBER_DATA_LENGTH];
} sIDInfoDataGWT;

typedef enum {
    OUTPUT_VOLTAGE_220_127V,
    OUTPUT_VOLTAGE_230_115V,
    OUTPUT_VOLTAGE_240_120V,
    OUTPUT_VOLTAGE_254_127V,
    OUTPUT_VOLTAGE_INVALID,
} eOutputVoltage;

#pragma pack(push, 1)
typedef struct {
    uint16_t wPVStartUp;
    uint16_t wStartDelay;
    uint16_t wReconnectTime;
    uint16_t wMinGridVoltage;
    uint16_t wMaxGridVoltage;
    uint16_t wMinGridFrequency;
    uint16_t wMaxGridFrequency;
} sInverterConfig;

typedef struct {
    uint8_t bYear;
    uint8_t bMonth;
    uint8_t bDay;
    uint8_t bHour;
    uint8_t bMinute;
    uint8_t bSecond;
} sInverterRTCTime;
Q_DECLARE_METATYPE(sInverterRTCTime);
#pragma pack(pop)

typedef struct {
    eOutputVoltage eOutVoltage;
    uint16_t wStartDelay;
    uint16_t wReconnectTime;
    uint16_t wMinGridFrequency;
    uint16_t wMaxGridFrequency;
} sInverterConfigUI;

typedef struct  {
    QString name;
    qint32 baudRate;
    QString stringBaudRate;
    QSerialPort::DataBits dataBits;
    QString stringDataBits;
    QSerialPort::Parity parity;
    QString stringParity;
    QSerialPort::StopBits stopBits;
    QString stringStopBits;
    QSerialPort::FlowControl flowControl;
    QString stringFlowControl;
    QString erro_msg;
}Serial_Settings;

typedef enum {
    INDUTIVO,
    CAPACITIVO
} ePowerFactorMode;

typedef struct {
    ePowerFactorMode eMode;
    float fpowerFactor;
} sInverterPowerFactor;

typedef struct {
    uint16_t wPowerFactor;
    uint8_t bPowerFactorLow;
    uint8_t bPowerFactorHigh;
} sGWTPowerFactor;

typedef struct {
    uint8_t bGWEPowerFactorHigh;
    uint8_t bGWEPowerFactorLow;
} sGWEPowerFactor;
