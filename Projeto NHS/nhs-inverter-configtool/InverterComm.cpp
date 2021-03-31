#include "InverterComm.h"

#define INVERTER_VENDOR_ID 0x0084
#define INVERTER_PRODUCT_ID 0x0041
#define INVERTER_TIMER_POOL_DEVICE_TIME_MS 200
#define INVERTER_TIMER_POOL_RTC_TIME_TIMES (60000U / INVERTER_TIMER_POOL_DEVICE_TIME_MS)
#define INVERTER_POOL_EVENTS_TIME_MS 10000
#define INVERTER_SCAN_PORTS_TIME_MS 500
#define INVERTER_REG_BUFFER_SIZE 128
#define INVERTER_REG_GROWATT_BUFFER_SIZE 8
#define INVERTER_REG_GOODWE_WRITE        11
#define INVERTER_REG_GOODWE_WRITE_RTC    15

#define INVERTER_REGISTER_REQUEST_DATA_LENGTH 16
#define INVERTER_ADDRESS_CONFIRM_DATA_LENGTH 0
#define INVERTER_REMOVE_CONFIRM_DATA_LENGTH 0

#define INVERTER_PAYLOAD_FIRST_BYTE         0x00
#define INVERTER_SYNC_BYTE_MSB              0xAA
#define INVERTER_SYNC_BYTE_LSB              0x55
#define INVERTER_SYNC_FUNC_QUERY_BYTE       0x40
#define INVERTER_SYNC_FUNC_QUERY_ES_BYTE    0X01
#define INVERTER_SYNC_BYTE_MSB_OFFSET       0U
#define INVERTER_SYNC_BYTE_LSB_OFFSET       1U
#define INVERTER_FUNC_BYTE_OFFSET           5U
#define INVERTER_DATA_LENGTH_BYTE_OFFSET    6U
#define INVERTER_PROTOCOL_HEADER_SIZE 7U
#define INVERTER_PROTOCOL_HEADER_SIZE_PLUS_CRC 9
#define INVERTER_HID_INPUT_REPORT_LENGTH 32.0f
#define INVERTER_HID_REPORT_ID 0x00
#define POWER_FAC_GWT_OFFSET                10000U

#define INVERTER_DATA_LENGTH_BYTE    6
#define INVERTER_PROTOCOL_MIN_HEADER_SIZE 7

#define INVERTER_RD_FUNC_CODE_RESPONSE_BITMASK  0x80
#define INVERTER_RESP_RUNNING_INFO_5K0_LENGTH   0x2E
#define INVERTER_RESP_RUNNING_INFO_1K5_LENGTH   0x3C
#define INVERTER_RESPONSE_ID_INFO_LENGTH        0x40
#define INVERTER_RESPONSE_SETTING_INFO_LENGTH   0x0C
#define INVERTER_RESPONSE_RTC_TIME_INFO_LENGTH  0x06
#define INVERTER_GWE_AA55_CRC_HEADER_SIZE       9

#define GROWATT_MODELO_LENGTH                   0x04
#define GROWATT_RTC_TIME_INFO_LENGTH            0x0C
#define GROWATT_RESPONSE_AB_LENGTH              0x2A
#define GROWATT_RESPONSE_INFO_LENGTH            0x52
#define GROWATT_RESPONSE_START_VAC_LENGTH       0x0A

#define GROWATT_HEADER_CRC_SIZE                 5
#define GROWATT_FUNCT_OFFSET                    1U
#define GROWATT_AMOUNT_OFFSET                   2U
#define GROWATT_FUNCT_WRITE_OFFSET              3U


#define GOODWE_RESPONDE_SERIAL_LENGHT 0x10
#define GOODWE_RESPONSE_MODEL_LENGHT 0x0A
#define GOODWE_RESPONSE_INFO_LENGHT 0x2C
#define GOODWE_CONFIG_LENGHT 0x0A
#define GOODWE_RESPONSE_RTC_LENGHT 0x06

#define GOODWE_RESPONSE_RTC_YEAR 0x00
#define GOODWE_RESPONSE_RTC_MONTH 0x01
#define GOODWE_RESPONSE_RTC_DAY 0x02
#define GOODWE_RESPONSE_RTC_HOUR 0x03
#define GOODWE_RESPONSE_RTC_MINUTE 0x04
#define GOODWE_RESPONSE_RTC_SECOND 0x05

#define GOODWE_RESPONDE_MODEL_L0 0x00
#define GOODWE_RESPONDE_MODEL_L1 0x01
#define GOODWE_RESPONDE_MODEL_L2 0x02
#define GOODWE_RESPONDE_MODEL_L3 0x03
#define GOODWE_RESPONDE_MODEL_L4 0x04
#define GOODWE_RESPONDE_MODEL_L5 0x05
#define GOODWE_RESPONDE_MODEL_L6 0x06
#define GOODWE_RESPONDE_MODEL_L7 0x07
#define GOODWE_RESPONDE_MODEL_L8 0x08
#define GOODWE_RESPONDE_MODEL_L9 0x09

#define GOODWE_RESPONSE_DELAY_TIME 0x00
#define GOODWE_RESPONSE_DELAY_TIME_OFFSET 0x02
#define GOODWE_RESPONSE_VAC_LOW 0x02
#define GOODWE_RESPONSE_VAC_LOW_OFFSET 0x02
#define GOODWE_RESPONSE_VAC_HIGH 0x04
#define GOODWE_RESPONSE_VAC_HIGH_OFFSET 0x02
#define GOODWE_RESPONSE_FAC_LOW 0x06
#define GOODWE_RESPONSE_FAC_LOW_OFFSET 0x02
#define GOODWE_RESPONSE_FAC_HIGH 0x08
#define GOODWE_RESPONSE_FAC_HIGH_OFFSET 0x02

#define GOODWE_RESPONSE_ERROR_CODE 0x00
#define GOODWE_RESPONSE_ERROR_CODE_OFFSET 0x04
#define GOODWE_RESPONSE_ENERGY_TOTAL 0x04
#define GOODWE_RESPONSE_ENERGY_TOTAL_OFFSET 0x04
#define GOODWE_RESPONSE_HOUR_TOTAL 0x08
#define GOODWE_RESPONSE_HOUR_TOTAL_OFFSET 0x04
#define GOODWE_RESPONSE_PV_VOLTAGE_1 0x0C
#define GOODWE_RESPONSE_PV_VOLTAGE_1_OFFSET 0x02
#define GOODWE_RESPONSE_PV_VOLTAGE_2 0x0E
#define GOODWE_RESPONSE_PV_VOLTAGE_2_OFFSET 0x02
#define GOODWE_RESPONSE_PV_CURRENT_1 0x10
#define GOODWE_RESPONSE_PV_CURRENT_1_OFFSET 0x02
#define GOODWE_RESPONSE_PV_CURRENT_2 0x12
#define GOODWE_RESPONSE_PV_CURRENT_2_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_VOLTAGE_PHASE1 0x14
#define GOODWE_RESPONSE_GRID_VOLTAGE_PHASE1_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_VOLTAGE_PHASE2 0x16
#define GOODWE_RESPONSE_GRID_VOLTAGE_PHASE2_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_VOLTAGE_PHASE3 0x18
#define GOODWE_RESPONSE_GRID_VOLTAGE_PHASE3_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_CURRENT_PHASE1 0x1A
#define GOODWE_RESPONSE_GRID_CURRENT_PHASE1_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_CURRENT_PHASE2 0x1C
#define GOODWE_RESPONSE_GRID_CURRENT_PHASE2_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_CURRENT_PHASE3 0x1E
#define GOODWE_RESPONSE_GRID_CURRENT_PHASE3_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_FREQUENCY_PHASE1 0x20
#define GOODWE_RESPONSE_GRID_FREQUENCY_PHASE1_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_FREQUENCY_PHASE2 0x22
#define GOODWE_RESPONSE_GRID_FREQUENCY_PHASE2_OFFSET 0x02
#define GOODWE_RESPONSE_GRID_FREQUENCY_PHASE3 0x24
#define GOODWE_RESPONSE_GRID_FREQUENCY_PHASE3_OFFSET 0x02
#define GOODWE_RESPONSE_FEEDING_POWER_GRID 0x26
#define GOODWE_RESPONSE_FEEDING_POWER_GRID_OFFSET 0x02
#define GOODWE_RESPONSE_STATUS 0x28
#define GOODWE_RESPONSE_STATUS_OFFSET 0x02
#define GOODWE_RESPONSE_LOW_BYTE_FEEDING 0x26
#define GOODWE_RESPONSE_TEMPERATURE 0x2A
#define GOODWE_RESPONSE_TEMPERATURE_OFFSET 0x02

#define GOODWE_RESPONDE_SERIAL_L0 0x00
#define GOODWE_RESPONDE_SERIAL_L1 0x01
#define GOODWE_RESPONDE_SERIAL_L2 0x02
#define GOODWE_RESPONDE_SERIAL_L3 0x03
#define GOODWE_RESPONDE_SERIAL_L4 0x04
#define GOODWE_RESPONDE_SERIAL_L5 0x05
#define GOODWE_RESPONDE_SERIAL_L6 0x06
#define GOODWE_RESPONDE_SERIAL_L7 0x07
#define GOODWE_RESPONDE_SERIAL_L8 0x08
#define GOODWE_RESPONDE_SERIAL_L9 0x09

#define GOODWE_HIGH_BYTE_AMOUNT     0x00
#define GOODWE_LOW_BYTE_AMOUNT      0x01
#define GOODWE_LOW_BYTE_AMOUNT_RTC  0x03
#define GOODWE_AMOUNT_BYTES         0x02
#define GOODWE_AMOUNT_BYTES_RTC     0x06
#define GOODWE_HEADER_CRC_QNT       5
#define GOODWE_MOD_FUNCTION_OFFSET  1U
#define GOODWE_MOD_AMOUNT_OFFSET  2U

#define GOODWE_FUNC_HIGH_OFFSET    2U
#define GOODWE_FUNC_LOW_OFFSET     3U


#define INV_GWT_AMOUNT_OF_DATAS_OFFSET      2
#define INV_SUCESS_WRITE_RETURN     0x06

#define INV_ADDR_OFFSET             0U
#define INV_ADDR_RANGE_MIN             1U
#define INV_ADDR_RANGE_MAX             247U

#define MSLEEP_TIME_TO_WRITE        100
#define MSLEEP_TIME_TO_WAIT         30
#define READY_READ_TIME_TO_WAIT     500
#define ISINVERTER_TIME_TO_WAIT     500
#define READY_READ_WHILE_TIME       50
#define DEBUG 1
#define DEVICE 0
#define REGISTER_INVERTER 0
#define LOGG 1
#define MODBUSTEST 0
static enum register_states { UNREGISTERED,
    ALLOCATE,
    REGISTERED,
    REMOVE,
    MAX_REGISTER_STATES } register_current_state;
static enum register_events { REGISTER_REQUEST,
    ADDRESS_CONFIRM,
    REMOVE_REQUEST,
    REMOVE_CONFIRM,
    MAX_REGISTER_EVENTS } register_new_event;

static QList<register_events> regEvents;

#if REGISTER_INVERTER
static sAllocateAddressData sInvAllocateAddrData = {
    { 0 },
    0x03
};
#else
static sAllocateAddressData sInvAllocateAddrData = {
    { 0 },
    INV_INITIAL_SLAVE_ADDRESS
};
#endif

static sIDInfoDataGWE sInvIDInfoData;
static sIDInfoDataGWT sInvIDInfoDataGWT;
static sRunningInfoData sInvRunningInfoData;
static sGWTRunInfoData sInvGWTRunningInfoData;
static sInverterConfigUI sInvCurrentConfigUI;
static sGWTPowerFactor sGWTpowFactor;
static sGWEPowerFactor sGWEpowFactor;
static sInverterConfig sInvCurrentConfig;
static sInverterRTCTime sInvCurrentRTCTime;
static InverterComm* ptrThis = nullptr;
static bool goodWe_GroWatt = false; // goodWe = true. growatt = false
static bool modbus_goodWe = false; // usb = true  ... rs485 = false
static bool firstUpdateOk = false;
static bool firstLineLog = true;
static int cWarning = 0, cError = 0, cInfo = 0, cDebug = 0, cCritical = 0;
static bool timeStart = false, valLow = false, vacHigh = false, timeRec = false;
static bool upyear = false, upmes = false, updia = false, uphour = false, upmin = false;
static bool upPassType = false, upPass1 = false, upPass2 = false, upPass3 = false, upValuePF = false , upCMDPF = false, upPFModel = false;
static uint8_t GwtMasterAddr = INV_GROWATT_COM_ADD;
#if MODBUSTEST
static uint8_t ModBusMasterAddr = 0xC9;
#else
static uint8_t ModBusMasterAddr = INV_GROWATT_COM_ADD;
#endif
InverterComm::InverterComm(QObject* parent)
    : QThread(parent)
{
    ptrThis = this;
#if DEBUG
    qDebug() << "começo do construtor";
#endif
#if LOGG
    qInstallMessageHandler(myMessageOutput);
#endif
    mSystemTime = QDateTime(QDate::currentDate(), QTime::currentTime());
    pPoolTimer = new QTimer(this);
    connect(pPoolTimer, SIGNAL(timeout()), this, SLOT(timerPoolDeviceCallback()), Qt::DirectConnection);

}

InverterComm::~InverterComm()
{
}

void InverterComm::registerConnection(bool registered)
{
    this->bInvRegistered = registered;
}

int InverterComm::initDeviceLayer(Serial_Settings settings)
{
    QMutex handle;
    if (this->bDevConnected)
        return CONNECTED;

    int status = 0;
    GwtMasterAddr = INV_GROWATT_COM_ADD;
    porta_serial = new QSerialPort(this);
    QString* pStatStr;
    porta_serial->setPortName(settings.name);
    porta_serial->setBaudRate(settings.baudRate);
    porta_serial->setDataBits(settings.dataBits);
    porta_serial->setParity(settings.parity);
    porta_serial->setStopBits(settings.stopBits);
    porta_serial->setFlowControl(settings.flowControl);
    if (porta_serial->open(QIODevice::ReadWrite)) {
        if(!IsInverter()){

        porta_serial->close();
            pStatStr = new QString(INV_COMM_INVERTER_NOT_FOUND_TEXT);
            status = INV_ERROR;
        }else {
#if DEBUG
        qDebug() << "Porta serial conectada com sucesso";
#endif
        this->bDevConnected = true;
        this->bDevId = 544; // so para ser diferente de zero
        handle.lock();
        getInverterConfig();
       // readDevice();
        getInverterAbout();
      //  readDevice();
        getInverterRTC();
      //  readDevice();
        getInverterInfo();
      //  readDevice();
        if(!goodWe_GroWatt || (goodWe_GroWatt && modbus_goodWe == false)){
            getPowerFactor();
         //   readDevice();
        }
        handle.unlock();
        firstUpdateOk = true; // Libera atualização do info
        pPoolTimer->start(INVERTER_TIMER_POOL_DEVICE_TIME_MS);
        pStatStr = new QString(INV_COMM_INVERTER_CONNECTED_TEXT);
#if DEBUG
        qDebug() << "ok";
        qDebug() << "Device connected succesfully!";
#endif
//        if(goodWe_GroWatt) {
//            this->bRead(INV_RD_FUNC_CODE_QUERY_ID_INFO);
//            msleep(500);
//            porta_serial->waitForReadyRead(1000);
//            this->bRead(INV_RD_FUNC_CODE_READ_RTC_TIME_VALUE);
//            msleep(500);
//            porta_serial->waitForReadyRead(1000);
//            status = CONNECTED;
//        } else {
//            // quando for growatt
//#if DEBUG
//            qDebug() << "IsDeviceGrowatt";
//#endif
//            this->bReadGT_holding(GROWATT_RTC_LO);
//            msleep(1500);
//            porta_serial->waitForReadyRead(1000);
//            this->bReadGT_holding(GROWATT_AB_LO);
//            msleep(1500);
//            porta_serial->waitForReadyRead(1000);
//            status = CONNECTED;
//        }
        }
    }
    else {
        pStatStr = new QString(PORT_NOT_FOUND_TEXT);
#if DEBUG
        qDebug() << "Device not connected succesfully!";
        qDebug() << porta_serial->errorString();
#endif
        status = PORT_ERROR;

    }
    /***********************************************************************************************************
    psHandle = new QHidApi(this);
    QList<QHidDeviceInfo> device = psHandle->enumerate(INVERTER_VENDOR_ID, INVERTER_PRODUCT_ID);

    if (device.count() <= 0)
    {
#if DEBUG
        qDebug() << "Device not found.";
#endif
        pStatStr = new QString(INV_COMM_INVERTER_NOT_FOUND_TEXT);
    } else
    {
#if DEBUG
        qDebug() << "Manufacturer String: " << device.at(0).manufacturerString;
        qDebug() << "Product String: " << device.at(0).productString;
        qDebug() << "Serial Number String: " << device.at(0).serialNumber;
        qDebug() << "VID/PID: " << device.at(0).vendorId << ":" << device.at(0).productId << hex;
#endif
        this->bDevId = psHandle->open(device.first().vendorId, device.first().productId, device.first().serialNumber);
        if (this->bDevId) {
            psHandle->setNonBlocking(this->bDevId); // set the read() to be non-blocking.
            pPoolTimer->start(INVERTER_TIMER_POOL_DEVICE_TIME_MS);
            this->bDevConnected = true;
            pStatStr = new QString(INV_COMM_INVERTER_CONNECTED_TEXT);
#if DEBUG
            qDebug() << "Device connected succesfully!";
#endif
            this->bRead(INV_RD_FUNC_CODE_QUERY_ID_INFO);
            msleep(500);
            this->bRead(INV_RD_FUNC_CODE_READ_RTC_TIME_VALUE);
        } else
        {
#if DEBUG
            qDebug() << "Device not connected succesfully!";
#endif
            pStatStr = new QString(INV_COMM_INVERTER_CONNECT_ERROR_TEXT);
        }
    }
*****************************************************************************/
    if (pStatStr != nullptr) {
        emit updateStatusBar(*pStatStr);
    }
#if DEBUG
    qDebug() << "Fim inicialização da porta serial e da leitura de versao e RTC";
#endif

    return status;
}

bool InverterComm::deInitDeviceLayer()
{
#if DEBUG
            qDebug() << "DE INIT";
#endif

    if (bDevConnected) {
#if DEBUG
            qDebug() << "Esta TENTANDO SE desconectar";
#endif
        if (porta_serial->isOpen()) {
            porta_serial->close();
#if DEBUG
            qDebug() << "Fim da conexao com a porta serial";
#endif
        }
        if(pPoolTimer->isActive()) {
#if DEBUG
            qDebug() << "Se ativo parando relogio";
#endif
            pPoolTimer->stop();
        }
        bDevConnected = false;
        firstUpdateOk = false;
        bDevId = 0;
        emit updateStatusBar(INV_COMM_INVERTER_DISCONNECTED_TEXT);
    }
    return !bDevConnected;
}

bool InverterComm::bRegister(uint8_t bFuncCode)
{
    bool ret = false;
    static uint8_t abHIDBuffer[INVERTER_REG_BUFFER_SIZE];
#if DEBUG
    qDebug() << "Função Bregister";
#endif
    if ((this->bDevConnected) && (this->bDevId != 0)) {
        QByteArray* pDataArray;
        if(goodWe_GroWatt){
        if (bFuncCode == INV_REG_FUNC_CODE_ALLOCATE_REG_ADDRESS) {
            pDataArray = new QByteArray(reinterpret_cast<const char*>(&sInvAllocateAddrData), sizeof(sAllocateAddressData));
        }
        else {
            pDataArray = new QByteArray();
        }

        memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

        pcPacket = new ProtocolPacket(bMasterAddress, ((!this->bInvRegistered) && (bFuncCode != INV_REG_FUNC_CODE_REMOVE_REGISTER)) ? INV_INITIAL_SLAVE_ADDRESS : sInvAllocateAddrData.bInvAddress, INV_CNTL_CODE_REGISTER, bFuncCode, *pDataArray);
        if (!pcPacket->toBuffer(&abHIDBuffer[0], INVERTER_REG_BUFFER_SIZE)) {
            ret = false;
#if DEBUG
            qDebug() << "Error to parse message...";
#endif
        }
        else {
            int dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
#if DEBUG
            qDebug() << "Bytes written: " << dSentBytes;
#endif
        }
    } else {
    // se for growatt
}
        }
    return ret;
}

bool InverterComm::bRead(uint8_t bFuncCode)
{
    bool ret = true;
    static uint8_t sendBuffer[INVERTER_REG_BUFFER_SIZE];
#if DEBUG
            qDebug() << "Mandando msg" << bFuncCode;
#endif
    if ((this->bDevConnected) && (this->bDevId != 0)) {
        QByteArray* pDataArray = new QByteArray();
        memset(sendBuffer, 0x00, sizeof(sendBuffer));

        pcPacket = new ProtocolPacket(bMasterAddress, sInvAllocateAddrData.bInvAddress, INV_CNTL_CODE_READ, bFuncCode, *pDataArray);
        if (!pcPacket->toBuffer(&sendBuffer[0], INVERTER_REG_BUFFER_SIZE)) {
            ret = false;
#if DEBUG
            qDebug() << "Error to parse message...";
#endif
        }
        else {
            int dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(sendBuffer), sizeof(sendBuffer)));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
#if DEBUG
            qDebug() << "Bytes written: " << dSentBytes;
#endif

        }


    }
    return ret;
}

QDataStream& operator<<(QDataStream& out, const sInverterConfig& data)
{
    out << static_cast<quint16>(data.wPVStartUp) << static_cast<quint16>(data.wStartDelay) << static_cast<quint16>(data.wMinGridVoltage)
        << static_cast<quint16>(data.wMaxGridVoltage) << static_cast<quint16>(data.wMinGridFrequency) << static_cast<quint16>(data.wMaxGridFrequency);
    return out;
}

QDataStream& operator<<(QDataStream& out, const sInverterRTCTime& data)
{
    out << static_cast<quint8>(data.bYear) << static_cast<quint8>(data.bMonth) << static_cast<quint8>(data.bDay)
        << static_cast<quint8>(data.bHour) << static_cast<quint8>(data.bMinute) << static_cast<quint8>(data.bSecond);
    return out;
}

bool InverterComm::bExecute(uint8_t bFuncCode)
{
    bool ret = false;
    uint8_t bCtrlCodeTest = '\0';
    QMutex protect;
    pPoolTimer->stop();
    protect.lock();
    porta_serial->clear();
    porta_serial->clearError();
    static uint8_t abHIDBuffer[INVERTER_REG_BUFFER_SIZE];
#if DEBUG
    qDebug() << "bExecute -  executando";
#endif

    if ((this->bDevConnected) && (this->bDevId != 0)) {
#if DEBUG
    qDebug() << "bExecute -  conectado";
#endif
        QByteArray mDataArray;
      // protect.lock();
        if(goodWe_GroWatt){

        QDataStream mDataStream(&mDataArray, QIODevice::ReadWrite);

        if (bFuncCode == INV_EXC_FUNC_CODE_SET_SETTING) {
#if DEBUG
    qDebug() << "bExecute -  Config infos";
#endif
            bCtrlCodeTest = INV_CNTL_CODE_EXECUTE;
            mDataStream << sInvCurrentConfig;
        }
        else if (bFuncCode == INV_EXC_FUNC_CODE_SET_RTC_TIME) {
#if DEBUG
    qDebug() << "bExecute -  RTC infos";
#endif
            bCtrlCodeTest = INV_CNTL_CODE_EXECUTE_T;
            mDataStream << sInvCurrentRTCTime;
        }
        else if (bFuncCode == INV_EXC_FUNC_CODE_ADJUST_REACTIVE_PWR) {
#if DEBUG
    qDebug() << "bExecute -  Fator de potencia infos";
#endif
            bCtrlCodeTest = INV_CNTL_CODE_EXECUTE_T;
            mDataStream << sGWEpowFactor.bGWEPowerFactorHigh << sGWEpowFactor.bGWEPowerFactorLow;

        }
#if DEBUG
    qDebug() << "bExecute -  executando : code " << bCtrlCodeTest;
#endif
        memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

        // Send offline query data message
        pcPacket = new ProtocolPacket(bMasterAddress, sInvAllocateAddrData.bInvAddress, bCtrlCodeTest, bFuncCode, mDataArray);
        if (!pcPacket->toBuffer(&abHIDBuffer[0], INVERTER_REG_BUFFER_SIZE)) {
            ret = false;
#if DEBUG
            qDebug() << "Error to parse message...";
#endif
        }
        else {
#if DEBUG
            qDebug() << "Enviado pacote - execute";
#endif
            int dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
            ret = true;
#if DEBUG
            qDebug() << "Bytes written: " << dSentBytes;
#endif
           readDevice();
        }
        }
    }
    protect.unlock();
    pPoolTimer->start();

    return ret;
}

void InverterComm::timerPoolDeviceCallback(void)
{
    static uint32_t dTimes = 0;
    if (dTimes == (INVERTER_TIMER_POOL_RTC_TIME_TIMES - 1)) {
        getInverterRTC();
        dTimes = 0;
    }
    if(lostConnection()){
        emit connectionLost();
    }
    dTimes++;
}

void action_sUnregistered_eRegisterRequest(void)
{
    if(goodWe_GroWatt) {
    ptrThis->bRegister(INV_REG_FUNC_CODE_ALLOCATE_REG_ADDRESS);
} else {
    // se gor growatt
}
    register_current_state = ALLOCATE;
    register_new_event = REGISTER_REQUEST;
}

void action_sAllocate_eAddressConfirm(void)
{
    // Now device is registered sucessfully
    register_current_state = REGISTERED;
    register_new_event = ADDRESS_CONFIRM;
    ptrThis->registerConnection(true);
}

void action_sRegistered_eRemoveRequest(void)
{
    if(goodWe_GroWatt) {
    ptrThis->bRegister(INV_REG_FUNC_CODE_REMOVE_REGISTER);
    } else {
    // se for growatt
}
    register_current_state = REMOVE;
    register_new_event = REMOVE_REQUEST;
}

void action_sRemoveRegister_eRemoveConfirm(void)
{
    ptrThis->registerConnection(false);
    register_current_state = UNREGISTERED;
    register_new_event = REMOVE_CONFIRM;
}

void action_null(void)
{
}

register_events get_new_event(void)
{
    register_events eRet = MAX_REGISTER_EVENTS;

    if (!regEvents.isEmpty()) {
        eRet = regEvents.front();
        regEvents.pop_front();
    }

    return eRet;
}

void (*const register_table[MAX_REGISTER_STATES][MAX_REGISTER_EVENTS])(void) = {
    { action_sUnregistered_eRegisterRequest, action_null, action_null, action_null }, /* procedures for state UNREGISTERED */
    { action_null, action_sAllocate_eAddressConfirm, action_null, action_null }, /* procedures for state ALLOCATE */
    { action_null, action_null, action_sRegistered_eRemoveRequest, action_null }, /* procedures for state REGISTERED */
    { action_null, action_null, action_null, action_sRemoveRegister_eRemoveConfirm } /* procedures for state REMOVE */
};

void callRegisterFunc(register_states state, bool bRegistered)
{
    if(goodWe_GroWatt) {
    switch (state) {
    case UNREGISTERED: {
        if (!bRegistered){
            ptrThis->bRegister(INV_REG_FUNC_CODE_OFFLINE_QUERY);
        }
        break;
    }
    case ALLOCATE: {
        if (!bRegistered){
            ptrThis->bRegister(INV_REG_FUNC_CODE_ALLOCATE_REG_ADDRESS);
        }
        break;
    }
    case REMOVE: {
        if (bRegistered){
            ptrThis->bRegister(INV_REG_FUNC_CODE_REMOVE_REGISTER);
        }
        break;
    }
    case REGISTERED:
    case MAX_REGISTER_STATES:
        break;
        //default:
        //   break;
    }
    }
}

void InverterComm::unregisterConnection()
{
    action_sRegistered_eRemoveRequest();
    while (this->bInvRegistered);
}

void InverterComm::updatePowFactorConfig(sInverterPowerFactor cfg) {
    if(!goodWe_GroWatt) {
        sGWTpowFactor.wPowerFactor = static_cast<uint16_t>(cfg.fpowerFactor * POWER_FAC_GWT_OFFSET);
        switch (cfg.eMode) {
        case CAPACITIVO : {
            sGWTpowFactor.wPowerFactor = POWER_FAC_GWT_OFFSET - sGWTpowFactor.wPowerFactor;
            break;
        }
        case INDUTIVO : {
            sGWTpowFactor.wPowerFactor += POWER_FAC_GWT_OFFSET;
            break;
        }
        }
        sGWTpowFactor.bPowerFactorLow = static_cast<uint8_t>(sGWTpowFactor.wPowerFactor & 0xFF);
        sGWTpowFactor.bPowerFactorHigh =static_cast<uint8_t>((sGWTpowFactor.wPowerFactor >> 8) & 0xFF);
        upPassType = false; upPass1 = false; upPass2 = false; upPass3 = false; upValuePF = false ; upCMDPF = false; upPFModel = false;
        this->bExecuteGwt(GROWATT_POWER_FACTOR_LO);
        this->getPowerFactor();
        msleep(100);
    } else {
         if(modbus_goodWe){   
                sGWTpowFactor.wPowerFactor = static_cast<uint16_t>(cfg.fpowerFactor * 100);
                switch (cfg.eMode) {
                case CAPACITIVO : {
                    if(sGWTpowFactor.wPowerFactor < 50) {
                        sGWTpowFactor.wPowerFactor = 49;
                    } else {
                    sGWTpowFactor.wPowerFactor = 100 - sGWTpowFactor.wPowerFactor;
                    }
                    break;
                }
                case INDUTIVO : {
                    if(sGWTpowFactor.wPowerFactor < 50) {
                        sGWTpowFactor.wPowerFactor = 51;
                    }
                    break;
                }
                }
                sGWEpowFactor.bGWEPowerFactorLow = static_cast<uint8_t>(sGWTpowFactor.wPowerFactor & 0xFF);
                sGWEpowFactor.bGWEPowerFactorHigh =static_cast<uint8_t>((sGWTpowFactor.wPowerFactor >> 8) & 0xFF);
                bExecute(INV_EXC_FUNC_CODE_ADJUST_REACTIVE_PWR);
         } else {
             uint16_t aux = static_cast<uint16_t>(round((cfg.fpowerFactor * 100)));
            switch (cfg.eMode) {
                case CAPACITIVO : {
                    // leading
                    if(aux == 0 || aux == 100 || aux < 90) {
                        // unitario
                        aux = 100;
                    } else {
                        aux = static_cast<uint16_t>(round(aux));
                    }    
                    break;
                }
                case INDUTIVO : {
                    // lagging 0.99 - 0.9
                    if(aux == 0 || aux == 100 || aux < 90) {
                        // unitario
                        aux = 100;
                    } else {
                        aux = 100 - aux;
                    }  

                    break;
                }
         }
                sGWEpowFactor.bGWEPowerFactorLow = static_cast<uint8_t>(aux & 0xFF);
                sGWEpowFactor.bGWEPowerFactorHigh =static_cast<uint8_t>((aux >> 8) & 0xFF);
                this->bExecuteGW_modbus(GOODWE_RANGE_REAC_POWER);
                msleep(MSLEEP_TIME_TO_WAIT);
                this->getPowerFactor();
         }

    }
}

void InverterComm::updateInverterConfig(sInverterConfigUI cfg)
{
    sInvCurrentConfig.wMinGridFrequency = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
    sInvCurrentConfig.wMaxGridFrequency = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;
    sInvCurrentConfig.wPVStartUp = INV_SETTING_PV_START_VOLTAGE_VALUE;

    sInvCurrentConfig.wStartDelay = cfg.wStartDelay;
    sInvCurrentConfig.wReconnectTime = cfg.wReconnectTime;

    if (goodWe_GroWatt) {
        switch (cfg.eOutVoltage) {
        case OUTPUT_VOLTAGE_220_127V: {
            sInvCurrentConfig.wMinGridVoltage = INV_SETTING_220V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = INV_SETTING_220V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        case OUTPUT_VOLTAGE_230_115V: {
            sInvCurrentConfig.wMinGridVoltage = INV_SETTING_230V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = INV_SETTING_230V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        case OUTPUT_VOLTAGE_240_120V: {
            sInvCurrentConfig.wMinGridVoltage = INV_SETTING_240V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = INV_SETTING_240V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        case OUTPUT_VOLTAGE_254_127V: {
            sInvCurrentConfig.wMinGridVoltage = INV_SETTING_254V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = INV_SETTING_254V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        default:
            break;
        }
        if(modbus_goodWe){
            this->bExecute(INV_EXC_FUNC_CODE_SET_SETTING);
            this->getInverterConfig();
            //this->readDevice();
        } else {
#if DEBUG
            qDebug() << "\t---\tEscrita --- config --- goodwe - modbus";
#endif
            timeStart = false; valLow = false;    vacHigh = false;
            this->bExecuteGW_modbus(GOODWE_RECONNECT_TIME_WRITE);
            this->getInverterConfig();
          //  this->readDevice();
        }
    } else {
     // se for growatt
        sInvCurrentConfig.wMinGridFrequency = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
        sInvCurrentConfig.wMaxGridFrequency = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;
        sInvCurrentConfig.wStartDelay = cfg.wStartDelay;
        sInvCurrentConfig.wReconnectTime = cfg.wReconnectTime;
        switch (cfg.eOutVoltage) {
        case OUTPUT_VOLTAGE_220_127V: {
            sInvCurrentConfig.wMinGridVoltage = GWT_SETTING_220V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = GWT_SETTING_220V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        case OUTPUT_VOLTAGE_230_115V: {
            sInvCurrentConfig.wMinGridVoltage = GWT_SETTING_230V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = GWT_SETTING_230V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        case OUTPUT_VOLTAGE_240_120V: {
            sInvCurrentConfig.wMinGridVoltage = GWT_SETTING_240V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = GWT_SETTING_240V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        case OUTPUT_VOLTAGE_254_127V: {
            sInvCurrentConfig.wMinGridVoltage = GWT_SETTING_254V_MIN_GRID_VOLTAGE_VALUE;
            sInvCurrentConfig.wMaxGridVoltage = GWT_SETTING_254V_MAX_GRID_VOLTAGE_VALUE;
            break;
        }
        default:
            break;
        }
        timeStart = false; timeRec = false;
        valLow = false;    vacHigh = false;

        this->bExecuteGwt(GROWATT_VAC_TIME_START_LO);
        this->getInverterConfig();
        //this->readDevice();
//        this->readDevice();
//        this->getPowerFactor();
//        this->readDevice();
//        emit ptrThis->confirmNewSettings(0x06);
}
    }

void InverterComm::getInverterAbout()
{
#if DEBUG
    qDebug()<<"Pegando about";
#endif
//    porta_serial->clear();
    if(goodWe_GroWatt) {
        if(modbus_goodWe){
            this->bRead(INV_RD_FUNC_CODE_QUERY_ID_INFO);
           // porta_serial->waitForReadyRead(500);
        }else {
            this->bReadGWe_modbus(GOODWE_ABOUT_SERIAL);
            this->readDevice();
            this->bReadGWe_modbus(GOODWE_ABOUT_MODELO);
}
    } else {
    // se for growatt ....
        this->bReadGT_holding(GROWATT_AB_LO);
    }
    readDevice();
}

void InverterComm::getInverterRTC()
{
#if DEBUG
    qDebug()<<"Pegando RTC info";
#endif
    if(goodWe_GroWatt) {
        if(modbus_goodWe){
            this->bRead(INV_RD_FUNC_CODE_READ_RTC_TIME_VALUE);
           // porta_serial->waitForReadyRead(500);
        } else {
            this->bReadGWe_modbus(GOODWE_RTC_YEAR_MONTH);
}
        } else {
        this->bReadGT_holding(GROWATT_RTC_LO);
    }
   // sleep(1);
    readDevice();
}

void InverterComm::setInverterRTC()
{
#if DEBUG
    qDebug()<<"Setando RTC info";
#endif

    if(goodWe_GroWatt) {
        if(modbus_goodWe){
            this->bExecute(INV_EXC_FUNC_CODE_SET_RTC_TIME);
          //  porta_serial->waitForReadyRead(500);
        } else {
            this->bExecuteGW_modbus(GOODWE_RTC);
          //  porta_serial->waitForReadyRead(500);
}
        } else {
    // se for growatt ....
//       porta_serial->clear();
       this->bExecuteGwt(GROWATT_RTC_LO);
    }
//    sleep(1);
}


void InverterComm::getInverterInfo()
{
#if DEBUG
    qDebug()<<"Pegando info";
#endif

    if(goodWe_GroWatt) {
        if(modbus_goodWe){
            this->bRead(INV_RD_FUNC_CODE_QUERY_RUNNING_INFO);
         //   porta_serial->waitForReadyRead(500);
        } else {
          this->bReadGWe_modbus(GOODWE_ERROR_CODE_LO);
}
    } else {
//        porta_serial->clear();
        this->bReadGT_input(GROWATT_ALL_INFO_LO);
    }
    msleep(MSLEEP_TIME_TO_WAIT);
    readDevice();
}

void InverterComm::getInverterConfig()
{
#if DEBUG
    qDebug()<<"Pegando Config e reconnect time";
#endif
    usedGWTfunc = true;
    usedGWeModfunc = true;
    if(goodWe_GroWatt) {
        if(modbus_goodWe){
              this->bRead(INV_RD_FUNC_CODE_QUERY_SETTING_INFO);
           //   porta_serial->waitForReadyRead(500);
        } else {
                bReadGWe_modbus(GOODWE_RECONNECT_TIME);
           //     porta_serial->waitForReadyRead(500);
        }
    } else {
//      porta_serial->clear();
      getInverterReconnectTime();
     // this->readDevice();
      this->bReadGT_holding(GROWATT_VAC_TIME_START_LO);
   //   porta_serial->waitForReadyRead(500);


}
   msleep(MSLEEP_TIME_TO_WAIT);
   readDevice();
}

void InverterComm::getInverterReconnectTime()
{
#if DEBUG
    qDebug()<<"Pegando ReconnectTime";
#endif
    usedGWTfunc = true;
    if(goodWe_GroWatt) {
//      this->bRead(INV_RD_FUNC_CODE_QUERY_SETTING_INFO);
//      porta_serial->waitForReadyRead(500);
    } else {
//      porta_serial->clear();
//      this->bReadGT_holding(GROWATT_VAC_TIME_START_LO);
      this->bReadGT_holding(GROWATT_RECONNECT_TIME_LO);
    //  porta_serial->waitForReadyRead(500);

    }
    msleep(MSLEEP_TIME_TO_WAIT);
    readDevice();
}

void InverterComm::getPowerFactor()
{
#if DEBUG
     qDebug()<<"Pegando Power Factor";
#endif
    if(goodWe_GroWatt) {
        if(modbus_goodWe) {
                this->bRead(INV_RD_FUNC_CODE_READ_ES_SETTING_INFO);
           //     porta_serial->waitForReadyRead(500);
        }else {
            this->bReadGWe_modbus(GOODWE_POWER_FACTOR);
         //   porta_serial->waitForReadyRead(300);
        }
        } else {
//      porta_serial->clear();
        this->bReadGT_holding(GROWATT_POWER_FACTOR_LO);
}
    msleep(MSLEEP_TIME_TO_WAIT);
    readDevice();
}

void InverterComm::syncInverterRTC(QDateTime currentTime)
{
#if DEBUG
            qDebug() << "Sincronizando -- RTC";
#endif
    this->mSystemTime = currentTime;
    sInvCurrentRTCTime.bYear = static_cast<uint8_t>(static_cast<unsigned int>((mSystemTime.date().year())) - 2000U);
    sInvCurrentRTCTime.bMonth = static_cast<uint8_t>(mSystemTime.date().month());
    sInvCurrentRTCTime.bDay = static_cast<uint8_t>(mSystemTime.date().day());
    sInvCurrentRTCTime.bHour = static_cast<uint8_t>(mSystemTime.time().hour());
    sInvCurrentRTCTime.bMinute = static_cast<uint8_t>(mSystemTime.time().minute());
    sInvCurrentRTCTime.bSecond = static_cast<uint8_t>((mSystemTime.time().second()));
    QMutex inv_rtc;
    if(!goodWe_GroWatt) {
        inv_rtc.lock();
        upyear = false; upmes = false; updia = false;
        uphour = false; upmin = false;
        this->setInverterRTC();
        inv_rtc.unlock();
        this->getInverterRTC();
      //  readDevice();
    } else {
       // no caso de for goodwe
       inv_rtc.lock();
        upyear = false; upmes = false; updia = false;
       uphour = false; upmin = false;
       this->setInverterRTC();
       this->getInverterRTC();
      // readDevice();
       inv_rtc.unlock();
}
}

void InverterComm::run(void)
{
#if DEBUG
    qDebug() << "From worker thread: " << currentThreadId();
#endif
    exec();
}

void handleRegisterFSM(register_events new_event)
{
    static register_events event = MAX_REGISTER_EVENTS;
    event = new_event;

    if ((/*(event >= 0) &&*/ (event < MAX_REGISTER_EVENTS)) && (/*(register_current_state >= 0) &&*/ (register_current_state < MAX_REGISTER_STATES))) {
        register_table[register_current_state][event]();
    }
}

int InverterComm::exec()
{
    register_current_state = UNREGISTERED;
    QMutex msgHandle;

    while (true) {
#if REGISTER_INVERTER
        // Device registered.
        handleRegisterFSM(get_new_event());
        callRegisterFunc(register_current_state, this->bInvRegistered);

        if (this->bInvRegistered) {
            // Read ID Info data
            this->bRead(INV_RD_FUNC_CODE_QUERY_ID_INFO);
#if DEBUG
            // Read running info to update HMI
            qDebug() << "Inverter registered at address " << hex << sInvAllocateAddrData.bInvAddress;
            qDebug() << "Query info read.";
#endif
        }
        else {
#if DEBUG
            qDebug() << "Inverter not connected";
#endif
        }
        msleep(INVERTER_POOL_EVENTS_TIME_MS);
#else
// Query running info data
#if DEBUG
        qDebug() << "Realizando a atualização de informações";
        qDebug() << "Conectado ????" << this->bDevConnected << this->bDevId;
#endif
        if(this->bDevConnected && firstUpdateOk == true){
#if DEBUG
        qDebug() << "atualizando ..." << this->bDevId;
#endif
            msgHandle.lock();
            getInverterInfo();
           // readDevice();
            msgHandle.unlock();
        }
        msleep(INVERTER_POOL_EVENTS_TIME_MS);
//       const QByteArray data = porta_serial->readAll();
#if DEBUG
        //  qDebug() << "recebendo data:" << data.toHex();
        qDebug() << "Leitura para a atualização de informações na aba";
#endif
        //readDevice();
#endif
    }
}
// TODO: Todas as chamadas da sendData tem dois argumentos um do array mesmo e outro do tamanho
int InverterComm::sendData(QByteArray request)
{

    int saida = static_cast<int>(porta_serial->write(request));
#if DEBUG
    if(!request.isEmpty())
        qDebug() <<"[" <<  QDateTime::currentDateTime().toString("dd-MM-yyyy HH:mm:ss") << "]Escrevendo Date:" << request.toHex();
    qDebug() << "Verificando se há erros ?" << porta_serial->errorString() << "Bytes válidos" << porta_serial->bytesAvailable();
#endif
//    porta_serial->waitForBytesWritten(1000);
//    msleep(1000);
    /*porta_serial->waitForReadyRead();
    const QByteArray data = porta_serial->readAll();
    qDebug() << "recebendo data:" << data.toHex() ;
    */
    return saida;
}

void handleRegisterMessage(uint8_t bFuncCode, QByteArray mData)
{
#if DEBUG
    qDebug() << "Parse da mensagem de registro:";
#endif
    if(goodWe_GroWatt) {
    switch (bFuncCode) {
    case INV_REG_FUNC_CODE_REGISTER_REQUEST: {
        if (mData.length() == INVERTER_REGISTER_REQUEST_DATA_LENGTH) {
#if DEBUG
    qDebug() << "\t registrando request";
#endif
            memcpy(&sInvAllocateAddrData.bSerialNumber, mData.constData(), INV_SERIAL_NUMBER_LENGTH);
            regEvents.append(REGISTER_REQUEST);
        }
        break;
    }
    case INV_REG_FUNC_CODE_ADDRESS_CONFIRM: {
#if DEBUG
    qDebug() << "\t Confirmação de endereço";
#endif
        if (mData.length() == INVERTER_ADDRESS_CONFIRM_DATA_LENGTH) {
            regEvents.append(ADDRESS_CONFIRM);
        }
        break;
    }
    case INV_REG_FUNC_CODE_REMOVE_CONFIRM: {
#if DEBUG
    qDebug() << "\t registrando remoção de confirmação de endereço";
#endif
        if (mData.length() == INVERTER_REMOVE_CONFIRM_DATA_LENGTH) {
            regEvents.append(REMOVE_CONFIRM);
        }
        break;
    }
    default:
        break;
    }
   }
}

eOutputVoltage getOutputVoltageIndex(uint8_t* pbData)
{
    eOutputVoltage eVoltage = OUTPUT_VOLTAGE_INVALID;
    uint16_t wMinVoltage, wMaxVoltage;
    if(!modbus_goodWe) {
        wMinVoltage = static_cast<uint16_t>((pbData[GOODWE_RESPONSE_VAC_LOW] << 8) | pbData[GOODWE_RESPONSE_VAC_LOW + 1]);
        wMaxVoltage = static_cast<uint16_t>((pbData[GOODWE_RESPONSE_VAC_HIGH] << 8) | pbData[GOODWE_RESPONSE_VAC_HIGH + 1]);
        sInvCurrentConfigUI.wMinGridFrequency = static_cast<uint16_t>(((pbData[GOODWE_RESPONSE_FAC_LOW] << 8) | pbData[GOODWE_RESPONSE_FAC_LOW + 1]));
        sInvCurrentConfigUI.wMaxGridFrequency = static_cast<uint16_t>(((pbData[GOODWE_RESPONSE_FAC_HIGH] << 8) | pbData[GOODWE_RESPONSE_FAC_HIGH + 1]));
    } else {
        wMinVoltage = static_cast<uint16_t>((pbData[INV_SETTING_INFO_MIN_GRID_VOLTAGE_OFFSET] << 8) | pbData[INV_SETTING_INFO_MIN_GRID_VOLTAGE_OFFSET + 1]);
        wMaxVoltage = static_cast<uint16_t>((pbData[INV_SETTING_INFO_MAX_GRID_VOLTAGE_OFFSET] << 8) | pbData[INV_SETTING_INFO_MAX_GRID_VOLTAGE_OFFSET + 1]);
        sInvCurrentConfigUI.wMinGridFrequency = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
        sInvCurrentConfigUI.wMaxGridFrequency = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;

}
    if(goodWe_GroWatt){
        if ((wMinVoltage == INV_SETTING_220V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == INV_SETTING_220V_MAX_GRID_VOLTAGE_VALUE)) {
            eVoltage = OUTPUT_VOLTAGE_220_127V;
        }
        else if ((wMinVoltage == INV_SETTING_230V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == INV_SETTING_230V_MAX_GRID_VOLTAGE_VALUE)) {
            eVoltage = OUTPUT_VOLTAGE_230_115V;
        }
        else if ((wMinVoltage == INV_SETTING_240V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == INV_SETTING_240V_MAX_GRID_VOLTAGE_VALUE)) {
            eVoltage = OUTPUT_VOLTAGE_240_120V;
        }
        else if ((wMinVoltage == INV_SETTING_254V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == INV_SETTING_254V_MAX_GRID_VOLTAGE_VALUE)) {
            eVoltage = OUTPUT_VOLTAGE_254_127V;
        }
    }
    return eVoltage;
}



void handleReadMessage(uint8_t bFuncCode, QByteArray mData)
{
#if DEBUG
    qDebug() << "Parse da mensagem de read/leitura:";
#endif
    if(goodWe_GroWatt) {
        if (bFuncCode & INVERTER_RD_FUNC_CODE_RESPONSE_BITMASK) {
            switch (bFuncCode) {
                case INV_RD_FUNC_CODE_RESPONSE_RUNNING_INFO: {
#if DEBUG
            qDebug() << "\tResponse Running Info << Func" << bFuncCode;
            if(!mData.isEmpty()){
                qDebug() << "tamanho" << mData.length();
                qDebug() << "dado ::" << mData.toHex();
            }
#endif
                uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

                if ((mData.length() == INVERTER_RESP_RUNNING_INFO_5K0_LENGTH) || (mData.length() == INVERTER_RESP_RUNNING_INFO_1K5_LENGTH)) {
#if DEBUG
    qDebug() << "Tamanho do pacote OK";
#endif

#if 0
                sInvRunningInfoData.fPV1Voltage = ((float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PV1_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV1_VOLTAGE_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPV1Current = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PV1_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV1_CURRENT_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPV2Voltage = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PV2_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV2_VOLTAGE_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPV2Current = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PV2_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV2_CURRENT_OFFSET + 1])) * 0.1f);

                sInvRunningInfoData.fPhaseL1Voltage = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L1_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_VOLTAGE_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPhaseL2Voltage = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L2_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L2_VOLTAGE_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPhaseL3Voltage = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L3_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L3_VOLTAGE_OFFSET + 1])) * 0.1f);

                sInvRunningInfoData.fPhaseL1Current = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPhaseL2Current = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L2_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L2_CURRENT_OFFSET + 1])) * 0.1f);
                sInvRunningInfoData.fPhaseL3Current = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L3_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L3_CURRENT_OFFSET + 1])) * 0.1f);

                sInvRunningInfoData.fPhaseL1Frequency = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L1_FREQUENCY_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_FREQUENCY_OFFSET + 1])) * 0.01f);
                sInvRunningInfoData.fPhaseL2Frequency = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L2_FREQUENCY_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L2_FREQUENCY_OFFSET + 1])) * 0.01f);
                sInvRunningInfoData.fPhaseL3Frequency = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_PHASE_L3_FREQUENCY_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L3_FREQUENCY_OFFSET + 1])) * 0.01f);

                sInvRunningInfoData.eInvWorkMode = (eInverterWorkMode)((uint32_t)((pbRecvData[INV_RUNNING_INFO_WORK_MODE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_WORK_MODE_OFFSET + 1]));

                sInvRunningInfoData.fInternalTemp = (float)(((uint32_t)((pbRecvData[INV_RUNNING_INFO_INTERNAL_TEMP_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_INTERNAL_TEMP_OFFSET + 1])) * 0.1f);

                sInvRunningInfoData.dErrorMessage = (uint32_t)(((pbRecvData[INV_RUNNING_INFO_ERROR_MESSAGE_H_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_ERROR_MESSAGE_H_OFFSET + 1]) << 16);
                sInvRunningInfoData.dErrorMessage |= (uint32_t)((pbRecvData[INV_RUNNING_INFO_ERROR_MESSAGE_L_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_ERROR_MESSAGE_L_OFFSET + 1]);

                uint32_t dTempFeedHours = 0;
                dTempFeedHours = (uint32_t)(((pbRecvData[INV_RUNNING_INFO_TOTAL_ENERGY2GRID_H_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_TOTAL_ENERGY2GRID_H_OFFSET + 1]) << 16);
                dTempFeedHours |= (uint32_t)((pbRecvData[INV_RUNNING_INFO_TOTAL_ENERGY2GRID_L_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_TOTAL_ENERGY2GRID_L_OFFSET + 1]);

                sInvRunningInfoData.fTotalEnergy2Grid = (float)(dTempFeedHours * 0.1f);

                sInvRunningInfoData.dTotalFeedHours = (uint32_t)(((pbRecvData[INV_RUNNING_INFO_TOTAL_FEED_HOURS_H_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_TOTAL_FEED_HOURS_H_OFFSET + 1]) << 16);
                sInvRunningInfoData.dTotalFeedHours |= (uint32_t)((pbRecvData[INV_RUNNING_INFO_TOTAL_FEED_HOURS_L_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_TOTAL_FEED_HOURS_L_OFFSET + 1]);
#endif

                sInvRunningInfoData.fPV1Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PV1_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV1_VOLTAGE_OFFSET + 1])))) * 0.1f;
                sInvRunningInfoData.fPV1Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PV1_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV1_CURRENT_OFFSET + 1])))) * 0.1f;
                sInvRunningInfoData.fPV2Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PV2_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV2_VOLTAGE_OFFSET + 1])))) * 0.1f;
                sInvRunningInfoData.fPV2Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PV2_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PV2_CURRENT_OFFSET + 1])))) * 0.1f;
                sInvRunningInfoData.fPhaseL1Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PHASE_L1_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_VOLTAGE_OFFSET + 1])))) * 0.1f;
                sInvRunningInfoData.fPhaseL1Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PHASE_L2_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L2_VOLTAGE_OFFSET + 1])))) * 0.1f;

                sInvRunningInfoData.wLowByteFeedingPW = static_cast<uint16_t>(((pbRecvData[INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET + 1]));

                sInvRunningInfoData.bPowerFactor = static_cast<uint8_t>(((sInvRunningInfoData.wLowByteFeedingPW / (sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current)) * 100));

                sInvRunningInfoData.fPhaseL1Frequency = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PHASE_L3_VOLTAGE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L3_VOLTAGE_OFFSET + 1])))) * 0.01f;

                sInvRunningInfoData.eInvWorkMode = static_cast<eInverterWorkMode>(static_cast<uint32_t>(((pbRecvData[INV_RUNNING_INFO_PHASE_L2_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L2_CURRENT_OFFSET + 1])));

                sInvRunningInfoData.fInternalTemp = static_cast<float>(static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PHASE_L3_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L3_CURRENT_OFFSET + 1])))) * 0.1f;

                sInvRunningInfoData.dErrorMessage = static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PHASE_L1_FREQUENCY_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_FREQUENCY_OFFSET + 1]) << 16));
                sInvRunningInfoData.dErrorMessage |= static_cast<uint32_t>(((pbRecvData[INV_RUNNING_INFO_PHASE_L2_FREQUENCY_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L2_FREQUENCY_OFFSET + 1]));

                uint32_t dTempFeedHours = 0;
                dTempFeedHours = static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_PHASE_L3_FREQUENCY_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L3_FREQUENCY_OFFSET + 1]) << 16));
                dTempFeedHours |= static_cast<uint32_t>(((pbRecvData[INV_RUNNING_INFO_LB_FEEDING_POWER_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_LB_FEEDING_POWER_OFFSET + 1]));

                sInvRunningInfoData.fTotalEnergy2Grid = static_cast<float>(dTempFeedHours) * 0.1f;

                sInvRunningInfoData.dTotalFeedHours = static_cast<uint32_t>((((pbRecvData[INV_RUNNING_INFO_WORK_MODE_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_WORK_MODE_OFFSET + 1]) << 16));
                sInvRunningInfoData.dTotalFeedHours |= static_cast<uint32_t>(((pbRecvData[INV_RUNNING_INFO_INTERNAL_TEMP_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_INTERNAL_TEMP_OFFSET + 1]));
#if DEBUG
                qDebug() << "ATUALIZANDO";
                qDebug() << "PV1 Tensaão:" << sInvRunningInfoData.fPV1Voltage;
                qDebug() << "PV1 Corrente:" << sInvRunningInfoData.fPV1Current;
                qDebug() << "PV2 Tensaão:" << sInvRunningInfoData.fPV2Voltage;
                qDebug() << "PV2 Corrente:" << sInvRunningInfoData.fPV2Current;
                qDebug() << "Fase L1 Tensão:" << sInvRunningInfoData.fPhaseL1Voltage;
                qDebug() << "Fase L1 Corrente:" << sInvRunningInfoData.fPhaseL1Current;
                qDebug() << "Fase L1 Frequencia:" << sInvRunningInfoData.fPhaseL1Current;
                qDebug() << "Energia Status Rede:" << sInvRunningInfoData.fTotalEnergy2Grid;
                qDebug() << "Tempo Status Rede:" << sInvRunningInfoData.dTotalFeedHours;
#endif

                emit ptrThis->updateInfoTab(sInvRunningInfoData);
#if DEBUG
                qDebug() << " ------ SUPOSTAMENTE ATUALIZANDO -------";
#endif
            }
            break;
        }
        case INV_RD_FUNC_CODE_RESPONSE_ID_INFO: {
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == INVERTER_RESPONSE_ID_INFO_LENGTH) {
                memcpy(sInvIDInfoData.abFWVersion, &pbRecvData[GWE_INV_ID_INFO_FW_VERSION_OFFSET], GWE_INV_ID_INFO_FW_VERSION_DATA_LENGTH);
                memcpy(sInvIDInfoData.abModelName, &pbRecvData[GWE_INV_ID_INFO_MODEL_NAME_OFFSET], GWE_INV_ID_INFO_MODEL_NAME_DATA_LENGTH);
                memcpy(sInvIDInfoData.abManufacturer, &pbRecvData[GWE_INV_ID_INFO_MANUFACTURER_OFFSET], GWE_INV_ID_INFO_MANUFACTURER_DATA_LENGTH);
                memcpy(sInvIDInfoData.abSerialNumber, &pbRecvData[GWE_INV_ID_INFO_SERIAL_NUMBER_OFFSET], GWE_INV_ID_INFO_SERIAL_NUMBER_DATA_LENGTH);
                memcpy(sInvIDInfoData.abNomPVVoltage, &pbRecvData[GWE_INV_ID_INFO_PV_VOLTAGE_OFFSET], GWE_INV_ID_INFO_PV_VOLTAGE_DATA_LENGTH);
                memcpy(sInvIDInfoData.abInternalVersion, &pbRecvData[GWE_INV_ID_INFO_INTERNAL_VERSION_OFFSET], GWE_INV_ID_INFO_INTERNAL_VERSION_DATA_LENGTH);
                sInvRunningInfoData.bSafetyCountryCode = pbRecvData[GWE_INV_ID_INFO_SAFETY_COUNTRY_CODE_OFFSET];
#if DEBUG
                qDebug() << "Atualizando about";
#endif
                emit ptrThis->updateAboutTab(sInvIDInfoData);
            }
            break;
        }
        case INV_RD_FUNC_CODE_RESPONSE_SETTING_INFO: {
#if DEBUG
                qDebug() << "resposta settings -- tamanho do pacote" << mData.length();
#endif

            uint8_t* pwRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == INVERTER_RESPONSE_SETTING_INFO_LENGTH) {
                sInvCurrentConfigUI.wStartDelay = static_cast<uint16_t>((pwRecvData[INV_SETTING_INFO_TIME2CONNECT_GRID_OFFSET] << 8) | pwRecvData[INV_SETTING_INFO_TIME2CONNECT_GRID_OFFSET + 1]);
                sInvCurrentConfigUI.eOutVoltage = getOutputVoltageIndex(pwRecvData);

                emit ptrThis->updateConfigTab(sInvCurrentConfigUI);
#if DEBUG
                qDebug() << "ATUALIZANDO -- setting info ";
                qDebug() << "Start Delay:" << sInvCurrentConfigUI.wStartDelay;
                qDebug() << "eOutVoltage:" << sInvCurrentConfigUI.eOutVoltage;
#endif
            }
            break;
        }
        case INV_RD_FUNC_CODE_RESPONSE_RTC_TIME_VALUE: {
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
#if DEBUG
                qDebug() << "resposta RTC -- tamanho do pacote" << mData.length();
#endif
            if (mData.length() == INVERTER_RESPONSE_RTC_TIME_INFO_LENGTH) {
                sInvCurrentRTCTime.bYear = pbRecvData[GWE_INV_RTC_TIME_INFO_YEAR_OFFSET];
                sInvCurrentRTCTime.bMonth = pbRecvData[GWE_INV_RTC_TIME_INFO_MONTH_OFFSET];
                sInvCurrentRTCTime.bDay = pbRecvData[GWE_INV_RTC_TIME_INFO_DAY_OFFSET];
                sInvCurrentRTCTime.bHour = pbRecvData[GWE_INV_RTC_TIME_INFO_HOUR_OFFSET];
                sInvCurrentRTCTime.bMinute = pbRecvData[GWE_INV_RTC_TIME_INFO_MINUTE_OFFSET];
                sInvCurrentRTCTime.bSecond = pbRecvData[GWE_INV_RTC_TIME_INFO_SECOND_OFFSET];
#if DEBUG
                qDebug() << "ATUALIZANDO -- RTC info ";
                qDebug() << sInvCurrentRTCTime.bDay  << "/" << sInvCurrentRTCTime.bMonth << "/" << sInvCurrentRTCTime.bYear;
                qDebug() << sInvCurrentRTCTime.bHour << ":" << sInvCurrentRTCTime.bMinute << ":" << sInvCurrentRTCTime.bSecond ;
#endif

                emit ptrThis->updateCurrentRTCTime(sInvCurrentRTCTime, 2000U);
            }
        } break;
            case INV_RD_FUNC_CODE_RESPONSE_READ_ES_SETTING_INFO: {
                uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
    #if DEBUG
                    qDebug() << "resposta RTC -- tamanho do pacote" << mData.length();
    #endif
                if (mData.length() == INVERTER_RESPONSE_RTC_TIME_INFO_LENGTH) {
                    sGWEpowFactor.bGWEPowerFactorHigh = pbRecvData[GWE_INV_POWER_FACTOR_RESPONSE_ES_OFFSET];
                    sGWEpowFactor.bGWEPowerFactorHigh= pbRecvData[GWE_INV_POWER_FACTOR_RESPONSE_ES_OFFSET+1];
    #if DEBUG
                    qDebug() << "ATUALIZANDO -- Power Factor info goodwe ";
                    qDebug() << sGWEpowFactor.bGWEPowerFactorHigh << sGWEpowFactor.bGWEPowerFactorLow;
    #endif
    float aux;
    aux = static_cast<float>(static_cast<uint16_t>((sGWEpowFactor.bGWEPowerFactorHigh << 8) | (sGWEpowFactor.bGWEPowerFactorLow)));
    aux = aux / 50;
#if DEBUG
                qDebug() << "ATUALIZANDO -- Power Factor info goodwe ";
                qDebug() << aux;
#endif
            emit ptrThis->updatePowerFactor(aux,goodWe_GroWatt);
                }
            } break;

        default:
            break;
        }
    }
    }
}

void handleExecuteMessage(uint8_t bFuncCode, QByteArray mData)
{
    (void)mData;
#if DEBUG
    qDebug() << "Parse Geral -> Parse Execute ->  message param";
    qDebug() << "Bfunc " << bFuncCode;
    if(!mData.isEmpty())
        qDebug() << mData.toHex();
#endif
 if (goodWe_GroWatt){
        if (bFuncCode & INVERTER_RD_FUNC_CODE_RESPONSE_BITMASK) {
            switch (bFuncCode) {
            case INV_EXC_FUNC_CODE_RESPONSE_SET_SETTING: {
                emit ptrThis->confirmNewSettings(static_cast<uint8_t>(mData.at(INVERTER_PAYLOAD_FIRST_BYTE)));
              //  ptrThis->bRead(INV_RD_FUNC_CODE_QUERY_SETTING_INFO);
                break;
            }
            case INV_EXC_FUNC_CODE_RESPONSE_SET_RTC_TIME: {
                emit ptrThis->confirmNewRTCTime(static_cast<uint8_t>(mData.at(INVERTER_PAYLOAD_FIRST_BYTE)));
               // ptrThis->bRead(INV_RD_FUNC_CODE_READ_RTC_TIME_VALUE);
                break;
            }
            case INV_EXC_FUNC_CODE_RESPONSE_ADJUST_REACTIVE_PWR: {
                emit ptrThis->confirmNewPowerFactor(true);
             //   ptrThis->bRead(INV_RD_FUNC_CODE_READ_ES_SETTING_INFO);
                break;
            }


            default:
                break;
            }
        }
    }
}

void InverterComm::handleMessage(uint8_t bCtrlCode, uint8_t bFuncCode, QByteArray mData)
{
    (void)bFuncCode;
    (void)mData;

#if DEBUG
    qDebug() << "Parse Geral da mensagem";
#endif
   if(goodWe_GroWatt) {
    switch (bCtrlCode) {
    case INV_CNTL_CODE_REGISTER: {
#if DEBUG
        qDebug() << "\tParse Geral -> Parse Register";
#endif

        handleRegisterMessage(bFuncCode, mData);
        break;
    }
    case INV_CNTL_CODE_READ: {
#if DEBUG
        qDebug() << "\tParse Geral -> Parse Read";
#endif
        handleReadMessage(bFuncCode, mData);
        break;
    }
    case INV_CNTL_CODE_EXECUTE_T:
    case INV_CNTL_CODE_EXECUTE: {
#if DEBUG
        qDebug() << "\tParse Geral -> Parse Execute";
#endif
        handleExecuteMessage(bFuncCode, mData);
        break;
    }
    default:
        break;
    }
  }

}

bool InverterComm::readDevice(void)
{
    if (!bDevId || this->bDevConnected == false)
        return false;
    QByteArray mRecvData, tenta;
    QMutex msgHandle;
    static uint8_t bTimes = 0;
    int retry = 6; //Quantas vezes deve esperar e ler de novo se a msg for incompleta
#if DEBUG
    int contador = 1;
    qDebug() << "Inicio Read Device ==== ";
#endif

//    porta_serial->clear();
    if(porta_serial->waitForReadyRead(READY_READ_TIME_TO_WAIT)){
        mRecvData = porta_serial->readAll();
#if DEBUG
        qDebug() << "Entrou no if" << mRecvData.toHex();
#endif
        while((porta_serial->waitForReadyRead(READY_READ_WHILE_TIME))) {
            mRecvData.append(porta_serial->readAll());
#if DEBUG
            qDebug() << "Tentando" << contador;
            contador++;
#endif

    #if DEBUG
    #if DEVICE
        qDebug() << "Recieve data Vazio, mas devia ter algo ... tentando ler ";
        qDebug() << "recebendo data RECEIVE DEVICE:" << mRecvData.toHex() << "bytes avaliables" << porta_serial->bytesAvailable();
    #endif
    #endif

        }
#if DEBUG
    if(!mRecvData.isEmpty())
        qDebug() << "Lido: "<<mRecvData.toHex() << mRecvData.length();
#endif
#if DEBUG
#if DEVICE
    qDebug() << "recebendo data RECEIVE DEVICE:" << mRecvData.toHex() << "bytes avaliables" << porta_serial->bytesAvailable();
    qDebug() << "Leitura do equipamento";
#endif
#endif
    } else {
#if DEBUG
        qDebug() << " Não Entrou no if mas ta no else" << mRecvData.toHex();
#endif
        if(porta_serial->bytesAvailable() != 0) {
            mRecvData = porta_serial->readAll();
#if DEBUG
        qDebug() << "\t\t\t\t dados disponíveis" << mRecvData.toHex();
#endif
        }
    }

    mReadData.clear();
    mReadData.append(mRecvData);

#if DEBUG
    if(!mReadData.isEmpty())
        qDebug() << "Lido2: "<<mReadData.toHex() << mReadData.length();
#endif
    if(goodWe_GroWatt) {

            if(modbus_goodWe) {

            if (!mReadData.isEmpty()) {
                qDebug() << "\tlendo a mais";

               if(mReadData.length() <= INVERTER_PROTOCOL_MIN_HEADER_SIZE) {
#if DEBUG
        qDebug() << "\tlendo a mais tamanho menor que 8";
#endif
                    tenta.clear();
                    tenta = porta_serial->readAll();
                    mReadData.append(tenta);
            }
            if(mReadData.length() < INVERTER_DATA_LENGTH_BYTE) {
                return false;
            }
            retry = 6;
            qDebug() <<"TAMANhOOO " <<hex << static_cast<uint8_t>(mReadData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET));
                while((mReadData.length() < mReadData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET) + INVERTER_PROTOCOL_HEADER_SIZE_PLUS_CRC) && retry > 0) {
#if DEBUG
        qDebug() << "\tTamanho do pacote não confere << retrry: " << retry;
#endif
                    tenta.clear();
                    porta_serial->waitForReadyRead(MSLEEP_TIME_TO_WRITE);
                   // msleep(300);
                    tenta = porta_serial->readAll();
                    mReadData.append(tenta);
                    retry--;
#if DEBUG
        qDebug()<<"Addddd retru" <<tenta.toHex() << "T: "<< mReadData.length() << "|" << retry << mReadData.toHex();
#endif
            }
             if(mReadData.length() > INVERTER_PROTOCOL_MIN_HEADER_SIZE + 1) {


                if ((static_cast<uint8_t>(mReadData.at(INVERTER_SYNC_BYTE_MSB_OFFSET)) == INVERTER_SYNC_BYTE_MSB) && (static_cast<uint8_t>(mReadData.at(INVERTER_SYNC_BYTE_LSB_OFFSET)) == INVERTER_SYNC_BYTE_LSB) && (bTimes == 0)) {
                    bTimes = static_cast<uint8_t>(ceil(static_cast<double>(((static_cast<uint8_t>(mReadData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET)) + INVERTER_PROTOCOL_HEADER_SIZE) / INVERTER_HID_INPUT_REPORT_LENGTH))) - 1);
    #if DEBUG
        qDebug() << " Valor do mReadData: " << mReadData.toHex() << "btimes" << bTimes;
    #endif
                    msgHandle.lock();
    #if DEBUG
        qDebug() << "Mutex -  : " << mReadData.toHex() << "btimes" << bTimes << msgHandle.try_lock();
    #endif
                    handleReadyRead();
                    msgHandle.unlock();
                }
                else {
    #if DEBUG
        qDebug() << " Valor do mReadData: ELSE " << mReadData.toHex() << "btimes" << bTimes;
    #endif
                    msgHandle.lock();

    #if DEBUG
        qDebug() << "Mutex -  :ELSE " << mReadData.toHex() << "btimes" << bTimes << msgHandle.try_lock();
    #endif
                    handleReadyRead();
                    msgHandle.unlock();
                    if (bTimes > 0) {
                        bTimes--;
                    }
                }

             }
             }

            if ((!mReadData.isEmpty()) && (bTimes == 0)) {
                msgHandle.lock();
                handleReadyRead();
                msgHandle.unlock();
            }
            } else {
                /*
                 * PARA GOODWE Quando for modbus
                 */
                int t = retry;
                if(!mReadData.isEmpty()) {
                if(mReadData.length() < GOODWE_HEADER_CRC_QNT ) {
            #if DEBUG
                    qDebug() << "[MODBUS GWe] TAMANHO INCOMPATIVEL tentar ler mais um pouco so 4 bytes << " << (mReadData.length()) << mReadData.isEmpty();
                    if(!mReadData.isEmpty())
                        qDebug() << mReadData.toHex();
            #endif
                    mRecvData.clear();
                    mRecvData = porta_serial->readAll();
                    mReadData.append(mRecvData);
                }
        #if DEBUG
                qDebug() << "[MODBUS] TAMANHO:" << (mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET) + GOODWE_HEADER_CRC_QNT) << "==" << static_cast<uint8_t>(mReadData.length());
            #endif
                if((static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GOODWE_HEADER_CRC_QNT == static_cast<uint8_t>(mReadData.length()))
                        || static_cast<uint8_t>(mReadData.at(GOODWE_MOD_FUNCTION_OFFSET)) == GOODWE_WRITE_FUNCTION
                        || static_cast<uint8_t>(mReadData.at(GOODWE_MOD_FUNCTION_OFFSET)) == GOODWE_NOT_GOOD_FUNCTION
                        || static_cast<uint8_t>(mReadData.at(GOODWE_MOD_FUNCTION_OFFSET)) == GOODWE_FAULTY_FUNCTION
                        ) {
        //            mReadData.append(mRecvData);
        #if DEBUG
            qDebug() << " [[MODBUS GoodWe]]Valor do mRecvData: Tamanho recebido confere " << mRecvData.toHex();
            qDebug() << "[MODBUS]Ou eh 0x10 - retorno de escrita \n OU EH ALGUM ERRO (0x83 ou 0x90) -" << hex <<static_cast<uint8_t>(mReadData.at(1));
        #endif
                        msgHandle.lock();

        #if DEBUG
            qDebug() << "[MODBUS]Mutex -  : " << mReadData.toHex() << msgHandle.try_lock();
        #endif
                        handleReadyRead();
                        msgHandle.unlock();
                } else {
                    // se nao a quantidade certa em mRecvData entao se concatena com algo que se leu
        #if DEBUG
            qDebug() << " [[MODBUS]]]Valor do mRecvData: Tamanho recebido NÃO confere " << mRecvData.toHex();
        #endif
        //            mReadData.append(mRecvData);
                    while(t >= 0 && (static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GOODWE_HEADER_CRC_QNT > static_cast<uint8_t>(mReadData.length())) ) {
                       porta_serial->waitForReadyRead((INVERTER_TIMER_POOL_DEVICE_TIME_MS/retry)+(10 * (7-t)));
                        mRecvData.clear();
                        mRecvData = porta_serial->readAll();
                        if(!mRecvData.isEmpty()) {
                            mReadData.append(mRecvData);
        #if DEBUG
                qDebug() << " [MODBUS]TAMANHO,[MODBUS] T=:" << t <<  (mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET) + 5) << "==" << static_cast<uint8_t>(mReadData.length());
            #endif
                        }
                        t--;
                    }
                    if (static_cast<uint8_t>(mReadData.length()) > static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GOODWE_HEADER_CRC_QNT  ) {
                        uint8_t dif = static_cast<uint8_t>(mReadData.length()) - (static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GOODWE_HEADER_CRC_QNT);
                        mRecvData.remove(static_cast<uint8_t>(mReadData.length()) - dif, dif);

                    }
                    if(static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GOODWE_HEADER_CRC_QNT == static_cast<uint8_t>(mReadData.length())){

                        msgHandle.lock();
        #if DEBUG
            qDebug() << "Mutex -  : " << mReadData.toHex() << msgHandle.try_lock();
        #endif
                        handleReadyRead();
                        msgHandle.unlock();

                    }
                }
           }


                    /*
                    * fim modbus
                    */


            }
    } else {
        // se for growatt
        int t = retry;
        if(mReadData.length() < GROWATT_HEADER_CRC_SIZE ) {
    #if DEBUG
            qDebug() << " TAMANHO INCOMPATIVEL ";
            qDebug() << mRecvData.toHex();
    #endif
            mRecvData.clear();
            mRecvData = porta_serial->readAll();
            mReadData.append(mRecvData);
            if(mRecvData.isEmpty()){
                return false;
            }
        }

#if DEBUG
        qDebug() << " TAMANHO:" << (mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET) + GROWATT_HEADER_CRC_SIZE) << "==" << static_cast<uint8_t>(mReadData.length());
    #endif
        if((static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GROWATT_HEADER_CRC_SIZE == static_cast<uint8_t>(mReadData.length()))
                || (static_cast<uint8_t>(mReadData.at(GROWATT_FUNCT_OFFSET)) == GROWATT_PRESET_SINGLE_REG
                ||  static_cast<uint8_t>(mReadData.at(GROWATT_FUNCT_OFFSET)) == GROWATT_ERRO_READ_HOLDING
                ||  static_cast<uint8_t>(mReadData.at(GROWATT_FUNCT_OFFSET)) == GROWATT_ERRO_READ_INPUT
                ||  static_cast<uint8_t>(mReadData.at(GROWATT_FUNCT_OFFSET)) == GROWATT_ERRO_PRESET_SINGLE_REG
                ||  static_cast<uint8_t>(mReadData.at(GROWATT_FUNCT_OFFSET)) == GROWATT_ERRO_PRESET_MULT_REG
                    )) {
//            mReadData.append(mRecvData);
#if DEBUG
    qDebug() << " [GWT]Valor do mRecvData: Tamanho recebido confere " << mRecvData.toHex();
    qDebug() << "Ou eh 0x06 - retorno de escrita";
#endif
                msgHandle.lock();

#if DEBUG
    qDebug() << "Mutex -  : " << mReadData.toHex() << msgHandle.try_lock();
#endif
                handleReadyRead();
                msgHandle.unlock();
        } else {
            // se nao a quantidade certa em mRecvData entao se concatena com algo que se leu
#if DEBUG
    qDebug() << " [GWT]Valor do mRecvData: Tamanho recebido NÃO confere " << mRecvData.toHex();
#endif
//            mReadData.append(mRecvData);
            while(t >= 0 && (static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + 5 > static_cast<uint8_t>(mReadData.length())) ) {
                porta_serial->waitForReadyRead(INVERTER_TIMER_POOL_DEVICE_TIME_MS/retry);
                porta_serial->waitForReadyRead((INVERTER_TIMER_POOL_DEVICE_TIME_MS/retry)+(10 * (7-t)));
                mRecvData.clear();
                mRecvData = porta_serial->readAll();
                if(!mRecvData.isEmpty()) {
                    mReadData.append(mRecvData);
#if DEBUG
        qDebug() << " TAMANHO, T=:" << t <<  (mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET) + 5) << "==" << static_cast<uint8_t>(mReadData.length());
    #endif
                }
                t--;
            }
            if (static_cast<uint8_t>(mReadData.length()) > static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GROWATT_HEADER_CRC_SIZE) {
                uint8_t dif = static_cast<uint8_t>(mReadData.length()) - (static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GROWATT_HEADER_CRC_SIZE);
                mRecvData.remove(static_cast<uint8_t>(mReadData.length()) - dif, dif);

            }
            if(static_cast<uint8_t>(mReadData.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET)) + GROWATT_HEADER_CRC_SIZE == static_cast<uint8_t>(mReadData.length())){

                msgHandle.lock();
#if DEBUG
    qDebug() << "Mutex -  : " << mReadData.toHex() << msgHandle.try_lock();
#endif
                handleReadyRead();
                msgHandle.unlock();

            }
        }
    }

    return true;
}

void InverterComm::handleReadyRead(void)
{
#if DEBUG
    qDebug() << "Parse do pronto da leitura";
#endif
    bool bCheckedAddress = true;
    if(goodWe_GroWatt){
        if(modbus_goodWe) {
            ProtocolPacket* pRecv = new ProtocolPacket(mReadData);
            if(pRecv->isValid()){
                if (((pRecv->getCtrlCode() == INV_CNTL_CODE_REGISTER) && (pRecv->getFuncCode() == INV_REG_FUNC_CODE_ADDRESS_CONFIRM)) || (this->bInvRegistered)) {
                    bCheckedAddress = pRecv->checkAddresses(sInvAllocateAddrData.bInvAddress, this->bMasterAddress);
                }

                if (bCheckedAddress) {
                    handleMessage(pRecv->getCtrlCode(), pRecv->getFuncCode(), pRecv->getData());
                }
            }
            delete pRecv;
        } else {
            ProtocolGoodWe* pGWRecv = new ProtocolGoodWe(mReadData);
#if DEBUG
    qDebug() << "[MODBUS] Parser";
#endif
    if(pGWRecv->isValid()){
#if DEBUG
    qDebug() << "[MODBUS]Checksum OK !!!";
#endif
            if(pGWRecv->checkAddressFunction(ModBusMasterAddr, pGWRecv->getInverterAdd())){
                handleMessageGoodWeModbus(bGWEfunc, pGWRecv->getFuncCode(), pGWRecv->getData(),mReadData);
                usedGWeModfunc = true;
#if DEBUG
    qDebug() << "Mensagem OK";
#endif
        }
    }
        delete pGWRecv;
    }
    } else{
        ProtocolGrowatt *pGTRecv = new ProtocolGrowatt(mReadData);
#if DEBUG
    qDebug() << "[growatt] Parser";
#endif
    if(pGTRecv->isValid()){
#if DEBUG
    qDebug() << "[growatt]Checksum OK !!!";
#endif
            if(pGTRecv->checkAddressFunction(GwtMasterAddr, pGTRecv->getInverterAdd())){
                handleMessageGrowatt(bGWTfunc, pGTRecv->getFuncCode(), pGTRecv->getData(),mReadData);
                usedGWTfunc = true;
#if DEBUG
    qDebug() << "Mensagem OK";
#endif
        }
    }
        delete pGTRecv;
    }

#if DEBUG
    if(!mReadData.isEmpty())
        qDebug() << mReadData.toHex();
    qDebug() << "Response received... " << mReadData.length() << "bytes.";
#endif
    mReadData.clear();
}

bool InverterComm::IsInverter()
{
//    porta_serial->clear();
    eDecideInverter decide_GoodWe_Growatt = INV_DEC_ERRO;
    bool original_conec = this->bDevConnected;
    uint orignal_ID = this->bDevId;
    this->bDevConnected = true;
    this->bDevId = 543; // teste
    bool isInverter = false;
    int retry = 3;
    QByteArray mRecvData, dataModelo, tenta;


    this->bReadGT_holding(GROWATT_MODELO_LO);
    if(porta_serial->waitForReadyRead(READY_READ_TIME_TO_WAIT)){
        mRecvData = porta_serial->readAll();
        while((porta_serial->waitForReadyRead(READY_READ_WHILE_TIME))) {
            mRecvData.append(porta_serial->readAll());
#if DEBUG
    qDebug() << "Tentando . .. ...";
#endif
        }
#if DEBUG
    if(!mRecvData.isEmpty()){
        qDebug() << "Lido IsInverter GWT: "<<mRecvData.toHex() << mRecvData.length();
        qDebug() << "Entrou no if IsInverter GWT" << mRecvData.toHex();
    }
#endif

    } else {
        if(porta_serial->bytesAvailable() != 0) {
            mRecvData = porta_serial->readAll();
#if DEBUG
    qDebug() << "ULTIMA TENTATIVA --- " << mRecvData.toHex();
#endif

        }
    }

#if DEBUG
    if(!mRecvData.isEmpty() && mRecvData.length() >= 3)
        qDebug() << "GWT recebendo data RECEIVE DEVICE:" << mRecvData.toHex() << "bytes avaliables" << porta_serial->bytesAvailable();
    qDebug() << "Leitura do equipamento";
#endif

    if(!mRecvData.isEmpty() && mRecvData.length() >= GROWATT_HEADER_CRC_SIZE){

        while((static_cast<uint8_t>(mRecvData.length()) < GROWATT_MODELO_LENGTH + GROWATT_HEADER_CRC_SIZE) && retry > 0 ) {
                    tenta.clear();
                    porta_serial->waitForReadyRead(READY_READ_WHILE_TIME * (retry+1));
                    tenta = porta_serial->readAll();
                    mRecvData.append(tenta);
                    retry--;
                }
        if( (mRecvData.at(GROWATT_FUNCT_OFFSET) == GROWATT_READ_HOLDING) &&
               (mRecvData.at(GROWATT_AMOUNT_OFFSET) == GROWATT_MODELO_LENGTH) &&
               (mRecvData.length() == (GROWATT_MODELO_LENGTH + GROWATT_HEADER_CRC_SIZE))) {
               // em tese eh growatt
               porta_serial->clearError();
               porta_serial->clear();
               goodWe_GroWatt = false;
               dataModelo = mRecvData.mid(3,GROWATT_MODELO_LENGTH);
#if DEBUG
    qDebug() << " DATA PARA ENVIO :" << dataModelo.toHex();
#endif
               decide_GoodWe_Growatt = INV_GROWATT;
           }
    }

#if MODBUSTEST

#else
    if(decide_GoodWe_Growatt == INV_DEC_ERRO){
           porta_serial->clearError();
           porta_serial->clear();
           mRecvData.clear();
           this->bRead(INV_RD_FUNC_CODE_READ_ES_SETTING_INFO);
           if(porta_serial->waitForReadyRead(READY_READ_TIME_TO_WAIT)){
               mRecvData = porta_serial->readAll();
               while((porta_serial->waitForReadyRead(READY_READ_WHILE_TIME))) {
                   mRecvData.append(porta_serial->readAll());
       #if DEBUG
           qDebug() << "Tentando . .. ...";
       #endif
               }
       #if DEBUG
           if(!mRecvData.isEmpty()){
               qDebug() << "Lido IsInverter GWE AA55: "<<mRecvData.toHex() << mRecvData.length();
               qDebug() << "Entrou no if IsInverter GWE AA55" << mRecvData.toHex();
           }
       #endif

           } else {
               if(porta_serial->bytesAvailable() != 0) {
                   mRecvData = porta_serial->readAll();
       #if DEBUG
           qDebug() << "ULTIMA TENTATIVA  GWE AA55--- " << mRecvData.toHex();
       #endif

               }
           }
    #if DEBUG
        if(!mRecvData.isEmpty())
            qDebug() << "GWE AA55 recebendo data RECEIVE DEVICE:" << mRecvData.toHex() << "bytes avaliables" << porta_serial->bytesAvailable();
        qDebug() << "Leitura do equipamento";
    #endif
        if(!mRecvData.isEmpty()) {
            retry = 3;
            while((static_cast<uint8_t>(mRecvData.length()) <  (static_cast<uint8_t>(mRecvData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET))+INVERTER_GWE_AA55_CRC_HEADER_SIZE)) && retry > 0 ) {
                tenta.clear();
                porta_serial->waitForReadyRead(READY_READ_WHILE_TIME * (retry+1));
                //porta_serial->waitForReadyRead();
                tenta = porta_serial->readAll();
                mRecvData.append(tenta);
                retry--;
            }
    #if DEBUG
        qDebug() << (static_cast<uint8_t>(mRecvData.at(INVERTER_SYNC_BYTE_MSB_OFFSET)) == INVERTER_SYNC_BYTE_MSB);
        qDebug() << (static_cast<uint8_t>(mRecvData.at(INVERTER_SYNC_BYTE_LSB_OFFSET)) == INVERTER_SYNC_BYTE_LSB);
        qDebug() << ((static_cast<uint8_t>(mRecvData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET))+INVERTER_GWE_AA55_CRC_HEADER_SIZE) == mRecvData.length());
        qDebug() << (static_cast<uint8_t>(mRecvData.at(INVERTER_FUNC_BYTE_OFFSET)) == INV_RD_FUNC_CODE_RESPONSE_READ_ES_SETTING_INFO);
        qDebug() << (static_cast<uint8_t>(mRecvData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET)) == INVERTER_SYNC_FUNC_QUERY_ES_BYTE);
    #endif
            if (
               (static_cast<uint8_t>(mRecvData.at(INVERTER_SYNC_BYTE_MSB_OFFSET)) == INVERTER_SYNC_BYTE_MSB) &&
               (static_cast<uint8_t>(mRecvData.at(INVERTER_SYNC_BYTE_LSB_OFFSET)) == INVERTER_SYNC_BYTE_LSB) &&
               ((static_cast<uint8_t>(mRecvData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET))+INVERTER_GWE_AA55_CRC_HEADER_SIZE) == mRecvData.length()) &&
               (static_cast<uint8_t>(mRecvData.at(INVERTER_FUNC_BYTE_OFFSET)) == INV_RD_FUNC_CODE_RESPONSE_READ_ES_SETTING_INFO) &&
               (static_cast<uint8_t>(mRecvData.at(INVERTER_DATA_LENGTH_BYTE_OFFSET)) == INVERTER_SYNC_FUNC_QUERY_ES_BYTE)
               )
            {
                goodWe_GroWatt = true;
                modbus_goodWe = true;
                dataModelo.clear();
                decide_GoodWe_Growatt = INV_GOODWE;

            }
        }
    }
#endif

    if(decide_GoodWe_Growatt == INV_DEC_ERRO) {

                porta_serial->clearError();
                porta_serial->clear();
                mRecvData.clear();
                this->bReadGWe_modbus(GOODWE_ABOUT_MODELO);
                if(porta_serial->waitForReadyRead(READY_READ_TIME_TO_WAIT)){
                    mRecvData = porta_serial->readAll();
                    while((porta_serial->waitForReadyRead(READY_READ_WHILE_TIME))) {
                        mRecvData.append(porta_serial->readAll());
            #if DEBUG
                qDebug() << "Tentando . .. ...";
            #endif
                    }
            #if DEBUG
                if(!mRecvData.isEmpty()){
                    qDebug() << "Lido IsInverter GWE MODBUS: "<<mRecvData.toHex() << mRecvData.length();
                    qDebug() << "Entrou no if IsInverter GWE MODBUS" << mRecvData.toHex();
                }
            #endif

                } else {
                    if(porta_serial->bytesAvailable() != 0) {
                        mRecvData = porta_serial->readAll();
            #if DEBUG
                qDebug() << "ULTIMA TENTATIVA  GWE MODBUS--- " << mRecvData.toHex();
            #endif

                    }
                }
                retry = 3;
                while((static_cast<uint8_t>(mRecvData.length()) < GOODWE_ABOUT_MODELO_LENGTH + 5) && retry > 0 ) {
                    tenta.clear();
                    porta_serial->waitForReadyRead(READY_READ_WHILE_TIME * (retry+1));
                    //porta_serial->waitForReadyRead();
                    tenta = porta_serial->readAll();
                    mRecvData.append(tenta);
                    retry--;
                }
 #if DEBUG
     if(!mRecvData.isEmpty())
        qDebug() << "GWE [modbus] recebendo data RECEIVE DEVICE:" << mRecvData.toHex() << "bytes avaliables" << porta_serial->bytesAvailable();
     qDebug() << "[ModBus]Leitura do equipamento";
     if(!mRecvData.isEmpty()){
     qDebug() << ((static_cast<uint8_t>(mRecvData.at(GOODWE_MOD_FUNCTION_OFFSET)) == GOODWE_READ_FUNCTION));
     qDebug() << (static_cast<uint8_t>(mRecvData.at(GOODWE_MOD_AMOUNT_OFFSET)) == GOODWE_ABOUT_MODELO_LENGTH);
     qDebug() << (static_cast<uint8_t>(mRecvData.length()) == GOODWE_ABOUT_MODELO_LENGTH + GOODWE_HEADER_CRC_QNT);
}
#endif
              if(!mRecvData.isEmpty()) {
                     if( (mRecvData.at(GOODWE_MOD_FUNCTION_OFFSET) == GOODWE_READ_FUNCTION) &&
                            (mRecvData.at(GOODWE_MOD_AMOUNT_OFFSET) == GOODWE_ABOUT_MODELO_LENGTH) &&
                            (mRecvData.length() == (GOODWE_ABOUT_MODELO_LENGTH + GOODWE_HEADER_CRC_QNT))) {
                            // em tese eh goodwe modbus
#if DEBUG
    qDebug() << "GWE [modbus] -- CONFIRMADO EH GOODWE MODBUS";
#endif
                         porta_serial->clearError();
                            porta_serial->clear();
                            goodWe_GroWatt = true;
                            modbus_goodWe = false;
                            dataModelo.clear();
                            decide_GoodWe_Growatt = INV_GOODWE_MOD;
                    }
                }
        }

    if(decide_GoodWe_Growatt == INV_DEC_ERRO) {
#if DEBUG
    qDebug() << " Não conseguiu enviar nada --- dec Erro";
#endif
        isInverter = false;
        this->bDevConnected = original_conec;
        this->bDevId = orignal_ID;
        dataModelo.clear();
        mRecvData.clear();
        emit ptrThis->prepareUI_goodWe_GWT(decide_GoodWe_Growatt, dataModelo);
        return isInverter;
    }else{
        isInverter = true;
    }

    if (decide_GoodWe_Growatt == INV_GROWATT || (decide_GoodWe_Growatt == INV_GOODWE_MOD)) {

        if(static_cast<uint8_t>(mRecvData.at(INV_ADDR_OFFSET)) >= INV_ADDR_RANGE_MIN && static_cast<uint8_t>(mRecvData.at(INV_ADDR_OFFSET)) <= INV_ADDR_RANGE_MAX) {
#if DEBUG
      qDebug() << "definindo novo Endereço" << mRecvData.toHex().at(INV_ADDR_OFFSET);
#endif
        if(decide_GoodWe_Growatt == INV_GROWATT){
#if DEBUG
      qDebug() << "\t\t\t para growatt" << mRecvData.toHex().at(INV_ADDR_OFFSET);
#endif
            GwtMasterAddr = static_cast<uint8_t>(mRecvData.at(INV_ADDR_OFFSET));
        }else {
#if DEBUG
      qDebug() << "\t\t\t para goodwe -- modbus" << mRecvData.toHex().at(INV_ADDR_OFFSET);
#endif
           ModBusMasterAddr = static_cast<uint8_t>(mRecvData.at(INV_ADDR_OFFSET));
        }
        }
    }
#if DEBUG
     qDebug() << "isInverter = " << isInverter << endl << "original" << original_conec;
#endif
    this->bDevConnected = original_conec;
    this->bDevId = orignal_ID;
    mRecvData.clear();
    emit ptrThis->prepareUI_goodWe_GWT(decide_GoodWe_Growatt, dataModelo);
    return (isInverter);
}

bool InverterComm::IsConnected(){
    return bDevConnected && pPoolTimer->isActive();
}

bool InverterComm::lostConnection(){
    bool ret = false;
    if(bDevConnected && porta_serial->error() == QSerialPort::ResourceError){
        ret = true;
    }
    return ret;
}

bool InverterComm::bReadGT_holding(uint8_t functCode) {

    bool ret = true;
#if DEBUG
            qDebug() << "[GROWATT] ... HOLDING REG";
#endif
    static uint8_t sendBuffer[INVERTER_REG_GROWATT_BUFFER_SIZE];
    if ((this->bDevConnected) && (this->bDevId != 0)) {
        memset(sendBuffer, 0x00, sizeof(sendBuffer));

        switch (functCode) {
        case GROWATT_RTC_LO: {
#if DEBUG
    qDebug() << "\t RTC";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_HOLDING,GROWATT_RTC_HI,GROWATT_RTC_LO,GROWATT_OFFSET_RTC_HI,GROWATT_OFFSET_RTC_LO);
            break;
        }
        case GROWATT_AB_LO: {
#if DEBUG
    qDebug() << "\t FW";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_HOLDING,GROWATT_AB_HI,GROWATT_AB_LO,GROWATT_OFFSET_AB_HI,GROWATT_OFFSET_AB_LO);
            break;
        }
        case GROWATT_VAC_TIME_START_LO: {
#if DEBUG
    qDebug() << "\t TIME START LOW e HIGH VAC";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_HOLDING,GROWATT_VAC_TIME_START_HI,GROWATT_VAC_TIME_START_LO,GROWATT_OFFSET_VAC_TIME_START_HI,GROWATT_OFFSET_VAC_TIME_START_LO);
            break;
        }
        case GROWATT_POWER_FACTOR_LO: {
#if DEBUG
    qDebug() << "\tPOWER Factor";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_HOLDING,GROWATT_POWER_FACTOR_HI,GROWATT_POWER_FACTOR_LO,GROWATT_OFFSET_POWER_FACTOR_HI,GROWATT_OFFSET_POWER_FACTOR_LO);
            break;
        }
        case GROWATT_RECONNECT_TIME_LO: {
#if DEBUG
    qDebug() << "\tRECONNECT TIME [0x77]";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_HOLDING,GROWATT_RECONNECT_TIME_HI,GROWATT_RECONNECT_TIME_LO,GROWATT_OFFSET_RECONNECT_TIME_HI,GROWATT_OFFSET_RECONNECT_TIME_LO);
            break;
        }
        case GROWATT_MODELO_LO: {
#if DEBUG
    qDebug() << "\t ADIQUIRINDO SOMENTE O MODELO E PÁ";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_HOLDING,GROWATT_MODELO_HI,GROWATT_MODELO_LO,GROWATT_OFFSET_MODELO_HI,GROWATT_OFFSET_MODELO_LO);
            break;
        }

        default : {
            return false;
        }

        }

        if (!gtpcPacket->toBufferGrowatt(&sendBuffer[0])) {
            ret = false;
#if DEBUG
            qDebug() << "Error to parse message...";
#endif
        }
        else {
            if(usedGWTfunc){
                bGWTfunc = gtpcPacket->getLowAddr();
                usedGWTfunc = false;
            }
            int dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(sendBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE ));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
#if DEBUG
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (sendBuffer);
#endif

        }

    }
    return ret;

}


bool InverterComm::bReadGT_input(uint8_t functCode) {

    bool ret = true;
#if DEBUG
            qDebug() << "[GROWATT] ... INPUT REG";
#endif
    static uint8_t sendBuffer[INVERTER_REG_GROWATT_BUFFER_SIZE]; // Continua o mesmo tamanho e pa
    if ((this->bDevConnected) && (this->bDevId != 0)) {
        memset(sendBuffer, 0x00, sizeof(sendBuffer));

        switch (functCode) {
        case GROWATT_ALL_INFO_LO: {
#if DEBUG
    qDebug() << "\t [GROWATT] ALL TAB INFO";
#endif
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_READ_INPUT,GROWATT_ALL_INFO_HI,GROWATT_ALL_INFO_LO,GROWATT_OFFSET_ALL_INFO_HI,GROWATT_OFFSET_ALL_INFO_LO);
            break;
        }

        default : {
            break;
        }

        }

        if (!gtpcPacket->toBufferGrowatt(&sendBuffer[0])) {
            ret = false;
#if DEBUG
            qDebug() << " [GROWATT] Info tab Error to parse message...";
#endif
        }
        else {
            if(usedGWTfunc){
                bGWTfunc = gtpcPacket->getLowAddr();
                usedGWTfunc = false;
            }
            int dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(sendBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
#if DEBUG
            qDebug() << " [GROWATT] Info tab ";
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (sendBuffer);
#endif

        }

    }
    return ret;

}

void handleReadHoldGrowatt(uint16_t lowReg, QByteArray mData)
{
#if DEBUG
    qDebug() << "Parse da mensagem de read holding:";
#endif


        switch (lowReg) {
        case GROWATT_AB_LO: {
#if DEBUG
            qDebug() << "\tResponse About Info (FW+SERIE+MODEL)";
            qDebug() << "tamanho" << mData.length();
            qDebug() << "dado:" << mData.toHex();
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == GROWATT_RESPONSE_AB_LENGTH) {
                memcpy(sInvIDInfoDataGWT.abFWVersion, &pbRecvData[GWT_INV_ID_INFO_FW_VERSION_OFFSET], GWT_INV_ID_INFO_FW_VERSION_DATA_LENGTH);
                memcpy(sInvIDInfoDataGWT.abModelName, &pbRecvData[GWT_INV_ID_INFO_MODEL_NAME_OFFSET], GWT_INV_ID_INFO_MODEL_NAME_DATA_LENGTH);
                memcpy(sInvIDInfoDataGWT.abSerialNumber, &pbRecvData[GWT_INV_ID_INFO_SERIAL_NUMBER_OFFSET], GWT_INV_ID_INFO_SERIAL_NUMBER_DATA_LENGTH);
#if DEBUG
                qDebug() << "Atualizando about";
#endif
                emit ptrThis->updateAboutTab(sInvIDInfoDataGWT);
            }
            break;
        }
        case GROWATT_RTC_LO: {
#if DEBUG
            qDebug() << "\tResponse Serial RTC Info";
            qDebug() << "tamanho" << mData.length();
            qDebug() << "dado:" << mData.toHex();
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
#if DEBUG
                qDebug() << "[GROWATT]resposta RTC -- tamanho do pacote" << mData.length();
#endif
            if (mData.length() == GROWATT_RTC_TIME_INFO_LENGTH) {
                uint16_t yearOffset = uint16_t(pbRecvData[GWT_INV_RTC_TIME_INFO_YEAR_OFFSET-1]<<8);
                sInvCurrentRTCTime.bYear = pbRecvData[GWT_INV_RTC_TIME_INFO_YEAR_OFFSET];
                sInvCurrentRTCTime.bMonth = pbRecvData[GWT_INV_RTC_TIME_INFO_MONTH_OFFSET];
                sInvCurrentRTCTime.bDay = pbRecvData[GWT_INV_RTC_TIME_INFO_DAY_OFFSET];
                sInvCurrentRTCTime.bHour = pbRecvData[GWT_INV_RTC_TIME_INFO_HOUR_OFFSET];
                sInvCurrentRTCTime.bMinute = pbRecvData[GWT_INV_RTC_TIME_INFO_MINUTE_OFFSET];
                sInvCurrentRTCTime.bSecond = pbRecvData[GWT_INV_RTC_TIME_INFO_SECOND_OFFSET];
#if DEBUG
                qDebug() << "ATUALIZANDO -- RTC info ";
                qDebug() << sInvCurrentRTCTime.bDay  << "/" << sInvCurrentRTCTime.bMonth << "/" << sInvCurrentRTCTime.bYear+yearOffset;
                qDebug() << sInvCurrentRTCTime.bHour << ":" << sInvCurrentRTCTime.bMinute << ":" << sInvCurrentRTCTime.bSecond ;
#endif

                emit ptrThis->updateCurrentRTCTime(sInvCurrentRTCTime, yearOffset);
            }
        } break;
        case GROWATT_VAC_TIME_START_LO: {
#if DEBUG
            qDebug() << "\tResponse Serial START HIGH e LOW VAC  Info";
            qDebug() << "tamanho" << mData.length();
            qDebug() << "dado:" << mData.toHex();
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
#if DEBUG
                qDebug() << "[GROWATT]resposta start time / vac -- tamanho do pacote" << mData.length();
#endif
            if (mData.length() == GROWATT_RESPONSE_START_VAC_LENGTH) {
                eOutputVoltage eVoltage = OUTPUT_VOLTAGE_INVALID;
                sInvCurrentConfigUI.wStartDelay = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_TIME_START_HI] << 8) | pbRecvData[GROWATT_OFFSET_TIME_START_HI + 1]);
                uint16_t wMinVoltage = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_TIME_START_HI+2] << 8) | pbRecvData[GROWATT_OFFSET_TIME_START_HI+3]);
                uint16_t wMaxVoltage = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_TIME_START_HI+4] << 8) | pbRecvData[GROWATT_OFFSET_TIME_START_HI+5]);
//                sInvCurrentConfigUI.wMinGridFrequency = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_TIME_START_HI+6] << 8) | pbRecvData[GROWATT_OFFSET_TIME_START_HI + 7]);
//                sInvCurrentConfigUI.wMaxGridFrequency = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_TIME_START_HI+8] << 8) | pbRecvData[GROWATT_OFFSET_TIME_START_HI + 9]);

                sInvCurrentConfigUI.wMinGridFrequency = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
                sInvCurrentConfigUI.wMaxGridFrequency = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;
                wMinVoltage = static_cast<uint16_t>(wMinVoltage);
                wMaxVoltage = static_cast<uint16_t>(wMaxVoltage);
                if ((wMinVoltage == GWT_SETTING_220V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == GWT_SETTING_220V_MAX_GRID_VOLTAGE_VALUE)) {
                    eVoltage = OUTPUT_VOLTAGE_220_127V;
                }
                else if ((wMinVoltage == GWT_SETTING_230V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == GWT_SETTING_230V_MAX_GRID_VOLTAGE_VALUE)) {
                    eVoltage = OUTPUT_VOLTAGE_230_115V;

                }
                else if ((wMinVoltage == GWT_SETTING_240V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == GWT_SETTING_240V_MAX_GRID_VOLTAGE_VALUE)) {
                    eVoltage = OUTPUT_VOLTAGE_240_120V;
                }
                else if ((wMinVoltage == GWT_SETTING_254V_MIN_GRID_VOLTAGE_VALUE) && (wMaxVoltage == GWT_SETTING_254V_MAX_GRID_VOLTAGE_VALUE)) {
                    eVoltage = OUTPUT_VOLTAGE_254_127V;
                }
                sInvCurrentConfigUI.eOutVoltage = eVoltage;
                emit ptrThis->updateConfigTab(sInvCurrentConfigUI);
#if DEBUG
                qDebug() << "[GROWATT] ATUALIZANDO -- setting info ";
                qDebug() << "Start Delay:" << sInvCurrentConfigUI.wStartDelay;
                qDebug() << "eOutVoltage:" << sInvCurrentConfigUI.eOutVoltage;
                qDebug() << "Max" << wMaxVoltage;
                qDebug() << "Min" << wMinVoltage;
#endif
            }
        } break;
        case GROWATT_POWER_FACTOR_LO: {
#if DEBUG
            qDebug() << "\tResponse (POWER FACTOR))";
            qDebug() << "tamanho" << mData.length();
            qDebug() << "dado:" << mData.toHex();
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
            uint16_t wpowerfactor;
            float power_factor;

            if (mData.length() == GROWATT_POWER_FACTOR_QT) {

                wpowerfactor = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_POWER_FACTOR_HI] << 8) | pbRecvData[GROWATT_OFFSET_POWER_FACTOR_HI + 1]);
                power_factor = static_cast<float> (wpowerfactor / 10000.0f);
#if DEBUG
                qDebug() << "Atualizando power factor";
                qDebug() << "WORD power factor" <<  wpowerfactor;
                qDebug() << "Float power factor" << power_factor;

#endif
                emit ptrThis->updatePowerFactor(power_factor,goodWe_GroWatt);

            }
            } break;
        case GROWATT_RECONNECT_TIME_LO: {
#if DEBUG
            qDebug() << "\tResponse (Reconnect Time))";
            qDebug() << "tamanho" << mData.length();
            qDebug() << "dado:" << mData.toHex();
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
            uint16_t wRectime;

            if (mData.length() == GROWATT_RECONNECT_TIME_QT) {

                wRectime = static_cast<uint16_t>((pbRecvData[GROWATT_OFFSET_RECONNECT_TIME_HI] << 8) | pbRecvData[GROWATT_OFFSET_RECONNECT_TIME_HI + 1]);
#if DEBUG
                qDebug() << "Atualizando ReconnectionTime";
                qDebug() << "WORD RectTime" <<  wRectime;

#endif
                emit ptrThis->updateRecTime(wRectime,goodWe_GroWatt);

            }
            } break;
        }

}

void handleReadInputGrowatt(QByteArray mData)
{
#if DEBUG
    qDebug() << "[GROWATT] Parse da mensagem de read input:";
#endif

#if DEBUG
            qDebug() << "\tResponse Info Tab (PVP + ERROR CODE + STATUS)";
            qDebug() << "tamanho" << mData.length();
            qDebug() << "dado:" << mData.toHex();
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == GROWATT_RESPONSE_INFO_LENGTH) {
#if DEBUG
                qDebug() << "Atualizando info tab";
#endif
                sInvGWTRunningInfoData.fPV1Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_PV1_VOLT_OFFSET] << 8) | pbRecvData[GWT_INV_PV1_VOLT_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPV1Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_PV1_CURR_OFFSET] << 8) | pbRecvData[GWT_INV_PV1_CURR_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPV2Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_PV2_VOLT_OFFSET] << 8) | pbRecvData[GWT_INV_PV2_VOLT_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPV2Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_PV2_CURR_OFFSET] << 8) | pbRecvData[GWT_INV_PV2_CURR_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPhaseL1Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_VAC1_OFFSET] << 8) | pbRecvData[GWT_INV_VAC1_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPhaseL1Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_IAC1_OFFSET] << 8) | pbRecvData[GWT_INV_IAC1_OFFSET + 1])))) * 0.1f;

                sInvGWTRunningInfoData.fPhaseL2Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_VAC2_OFFSET] << 8) | pbRecvData[GWT_INV_VAC2_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPhaseL2Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_IAC2_OFFSET] << 8) | pbRecvData[GWT_INV_IAC2_OFFSET + 1])))) * 0.1f;

                sInvGWTRunningInfoData.fPhaseL3Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_VAC3_OFFSET] << 8) | pbRecvData[GWT_INV_VAC3_OFFSET + 1])))) * 0.1f;
                sInvGWTRunningInfoData.fPhaseL3Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_IAC3_OFFSET] << 8) | pbRecvData[GWT_INV_IAC3_OFFSET + 1])))) * 0.1f;


                // LOW BYTE FEEDING ??? ENCONTRAR
                //sInvGWTRunningInfoData.wLowByteFeedingPW = static_cast<uint16_t>(((pbRecvData[INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET] << 8) | pbRecvData[INV_RUNNING_INFO_PHASE_L1_CURRENT_OFFSET + 1]));
                // Fator de potencia ??? ENCONTRAR
                //sInvGWTRunningInfoData.bPowerFactor = static_cast<uint8_t>(((sInvRunningInfoData.wLowByteFeedingPW / (sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current)) * 100));
                float freq;
                freq = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_GRID_FREQUENCY_OFFSET] << 8) | pbRecvData[GWT_INV_GRID_FREQUENCY_OFFSET + 1])))) * 0.01f;
                sInvGWTRunningInfoData.fPhaseL1Frequency = freq;
                sInvGWTRunningInfoData.fPhaseL2Frequency = freq;
                sInvGWTRunningInfoData.fPhaseL3Frequency = freq;


                sInvGWTRunningInfoData.eInvGWTWorkMode = static_cast<eInvGWTWorkMode_tDef>(static_cast<uint32_t>(((pbRecvData[GWT_INV_STATUS_OFFSET] << 8) | pbRecvData[GWT_INV_STATUS_OFFSET + 1])));

                sInvGWTRunningInfoData.fInternalTemp = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GWT_INV_TEMPERATURE_OFFSET] << 8) | pbRecvData[GWT_INV_TEMPERATURE_OFFSET + 1])))) * 0.1f;

                sInvGWTRunningInfoData.dErrorMessage = static_cast<uint16_t>((((pbRecvData[GWT_INV_FAULT_CODE_OFFSET] << 8) | pbRecvData[GWT_INV_FAULT_CODE_OFFSET + 1])));

                uint32_t dTempFeedHours = 0;
                float ftime = 0.0;
                dTempFeedHours = static_cast<uint32_t>((((pbRecvData[GWT_INV_ENE_TOTAL_HI_OFFSET] << 8) | pbRecvData[GWT_INV_ENE_TOTAL_HI_OFFSET + 1]) << 16));
                dTempFeedHours |= static_cast<uint32_t>(((pbRecvData[GWT_INV_ENE_TOTAL_LO_OFFSET] << 8) | pbRecvData[GWT_INV_ENE_TOTAL_LO_OFFSET + 1]));

#if DEBUG
                qDebug() << "em em ...... " << dTempFeedHours;
#endif
                sInvGWTRunningInfoData.fTotalEnergy2Grid = static_cast<float>(dTempFeedHours) * 0.1f;

                dTempFeedHours = static_cast<uint32_t>((((pbRecvData[GWT_INV_TIME_TOTAL_HI_OFFSET] << 8) | pbRecvData[GWT_INV_TIME_TOTAL_HI_OFFSET + 1]) << 16));
                dTempFeedHours |= static_cast<uint32_t>(((pbRecvData[GWT_INV_TIME_TOTAL_LO_OFFSET] << 8) | pbRecvData[GWT_INV_TIME_TOTAL_LO_OFFSET + 1]));
                dTempFeedHours = (static_cast<uint32_t>(dTempFeedHours / 2)); // total segundos
#if DEBUG
                qDebug() << "Tempo Status Rede: 1" << ftime;
#endif
                ftime = static_cast<float>(dTempFeedHours) / 60.0f; // total minutos
#if DEBUG
                qDebug() << "Tempo Status Rede: 2" << ftime;
#endif
                ftime = ftime / 60; // total horas
#if DEBUG
                qDebug() << "Tempo Status Rede: 3" << ftime;
#endif
                sInvGWTRunningInfoData.dTotalFeedHours = ftime;



#if DEBUG
                qDebug() << "ATUALIZANDO";
                qDebug() << "PV1 Tensão:" << sInvGWTRunningInfoData.fPV1Voltage;
                qDebug() << "PV1 Corrente:" << sInvGWTRunningInfoData.fPV1Current;
                qDebug() << "PV1 Tensão:" << sInvGWTRunningInfoData.fPV2Voltage;
                qDebug() << "PV1 Corrente:" << sInvGWTRunningInfoData.fPV2Current;

                qDebug() << "Fase L1 Tensão:" << sInvGWTRunningInfoData.fPhaseL1Voltage;
                qDebug() << "Fase L1 corrente:" << sInvGWTRunningInfoData.fPhaseL1Current;
                qDebug() << "Fase L2 Tensão:" << sInvGWTRunningInfoData.fPhaseL2Voltage;
                qDebug() << "Fase L2 corrente:" << sInvGWTRunningInfoData.fPhaseL2Current;
                qDebug() << "Fase L3 Tensão:" << sInvGWTRunningInfoData.fPhaseL3Voltage;
                qDebug() << "Fase L3 corrente:" << sInvGWTRunningInfoData.fPhaseL3Current;
                qDebug() << "Fase L1 Tensão:" << sInvGWTRunningInfoData.fPhaseL1Frequency;
                qDebug() << "em em ...... " << dTempFeedHours;
                qDebug() << "Energia Status Rede:" << sInvGWTRunningInfoData.fTotalEnergy2Grid;
                qDebug() << "Temperatura:" << sInvGWTRunningInfoData.fInternalTemp;
                qDebug() << "Erro:" << sInvGWTRunningInfoData.dErrorMessage;

#endif

                emit ptrThis->updateInfoTab(sInvGWTRunningInfoData);
#if DEBUG
                qDebug() << " ------ SUPOSTAMENTE ATUALIZANDO -------";
#endif


    }
}

bool valide_GWT_func_message(QByteArray recv, uint8_t low_func) {
#if DEBUG
                qDebug() << " ------ VALIDANDO e ATUALIZANDO -------";
                qDebug() << recv.toHex() << "LOW FUNC" << low_func;
                qDebug() << (recv.at(INV_ADDR_OFFSET) == GwtMasterAddr) << (recv.at(GROWATT_FUNCT_OFFSET) == GROWATT_PRESET_SINGLE_REG) << (recv.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET) == 0x00) << (static_cast<uint8_t>(recv.at(GROWATT_FUNCT_WRITE_OFFSET)) == low_func);
                qDebug() << static_cast<uint8_t>(recv.at(GROWATT_FUNCT_WRITE_OFFSET));
#endif
if(!recv.isEmpty()) {
    if(recv.at(INV_ADDR_OFFSET) == GwtMasterAddr &&
       recv.at(GROWATT_FUNCT_OFFSET) == GROWATT_PRESET_SINGLE_REG &&
       recv.at(INV_GWT_AMOUNT_OF_DATAS_OFFSET) == 0x00 &&
       static_cast<uint8_t>(recv.at(GROWATT_FUNCT_WRITE_OFFSET)) == low_func) {
#if DEBUG
                qDebug() << "\t\t\t TRUE!!! Tá valido";
#endif
        return true;
    }
}
    return false;

}

bool valide_GWE_func_message(QByteArray recv, uint16_t low_func) {
    uint16_t aux = static_cast<uint16_t>(((recv.at(GOODWE_FUNC_HIGH_OFFSET) << 8)) | recv.at(GOODWE_FUNC_LOW_OFFSET));
#if DEBUG
                qDebug() << " ------ VALIDANDO e ATUALIZANDO [MODBUS]-------";
                if(!recv.isEmpty()) {
                    qDebug() << recv.toHex() << "LOW FUNC" << low_func;
                    qDebug() << (static_cast<uint8_t>(recv.at(INV_ADDR_OFFSET)) == ModBusMasterAddr) << (recv.at(GOODWE_MOD_FUNCTION_OFFSET) == GOODWE_WRITE_FUNCTION) << aux << (static_cast<uint16_t>(aux) == low_func);
                    qDebug() << static_cast<uint8_t>(recv.at(GOODWE_FUNC_HIGH_OFFSET));
                  }
#endif

if(!recv.isEmpty()) {
    if(static_cast<uint8_t>(recv.at(INV_ADDR_OFFSET)) == static_cast<uint8_t>(ModBusMasterAddr) &&
       recv.at(GOODWE_MOD_FUNCTION_OFFSET) == GOODWE_WRITE_FUNCTION &&
       aux == low_func) {
#if DEBUG
                qDebug() << "\t\t\t TRUE!!! Tá valido";
#endif
        return true;
    }
}
    return false;

}

void handleExecuteGWTMessage(uint8_t lowReg, QByteArray mData)
{
    (void)mData;
    // depois tem que arrumar pois neste caso entra 3x e 6x na outra
    qDebug() << "-- execute -- " << mData.toHex();
    switch (lowReg) {

        case GROWATT_VAC_HIGH_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            vacHigh = true;
        }else {
            emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
        }

            break;
        }
        case GROWATT_VAC_TIME_START_LO:{
            if(valide_GWT_func_message(mData,lowReg)) {
                timeStart = true;
            }
            break;
    }
        case GROWATT_RECONNECT_TIME_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            timeRec = true;
//          ptrThis->getInverterReconnectTime();
        }
            break;
        }


        case GROWATT_VAC_LOW_LO: {
#if DEBUG
                qDebug() << "Mensagem de VAC - high confirm" << timeStart << valLow << vacHigh << timeRec;
#endif
        if(valide_GWT_func_message(mData,lowReg)) {
            valLow = true;
#if DEBUG
                qDebug() << "Mensagem de VAC - high confirm" << timeStart << valLow << vacHigh << timeRec;
#endif
            if(vacHigh && valLow && timeStart && timeRec){
                emit ptrThis->confirmNewSettings(INV_SUCESS_WRITE_RETURN);
//                ptrThis->getInverterConfig();

            } else {
                emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
            }
        }

            break;
        }
    case GROWATT_RTC_YEAR_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upyear = true;
        }
        break;
}
    case GROWATT_RTC_MONTH_LO: {
    if(valide_GWT_func_message(mData,lowReg)) {
        upmes = true;
    }
    break;
}
        case GROWATT_RTC_DAY_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            updia = true;
        }
        break;
}
    case GROWATT_RTC_HOUR_LO: {
            if(valide_GWT_func_message(mData,lowReg)) {
                uphour = true;
            }
            break;
    }
        case GROWATT_RTC_MINUTE_LO: {
                    if(valide_GWT_func_message(mData,lowReg)) {
                        upmin = true;
                    }

#if DEBUG
                qDebug() << upmin << uphour <<updia << upmes << upyear;
#endif
                    if(upmes && updia && upyear && uphour && upmin) {
                        emit ptrThis->confirmNewRTCTime(INV_SUCESS_WRITE_RETURN);
                        break;
                    } else {
                        emit ptrThis->confirmNewRTCTime(false);
                    }
                    break;
            }
// upPassType = false, upPass1 = false, upPass2 = false, upPass3 = false, upValuePF = false , upCMDPF = false, upPFModel = false
    case GROWATT_PASSWORD_TYPE_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upPassType = true;
        } else {
            upPassType = false;
}
        break;
    }
    case GROWATT_PASSWORD_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upPass1 = true;
        } else {
            upPass1 = false;
}
        break;
    }
    case GROWATT_PASSWORD2_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upPass2 = true;
        } else {
            upPass2 = false;
}
        break;
    }

    case GROWATT_PASSWORD3_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upPass3 = true;
        } else {
            upPass3 = false;
}
        break;
    }
    case GROWATT_POWER_FACTOR_CMD_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upCMDPF = true;
        } else {
            upCMDPF = false;
}
        break;
    }
    case GROWATT_POWER_FACTOR_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upValuePF = true;
        } else {
            upValuePF = false;
}
        break;
    }
    case GROWATT_PF_MODEL_LO: {
        if(valide_GWT_func_message(mData,lowReg)) {
            upPFModel = true;
        }
#if DEBUG
    qDebug() <<"PASS: " << upPassType << "&&" << upPass1 << "&&" << upPass2 << "&&" <<  upPass3 << "&&" << upValuePF << "&&" << upCMDPF << "&&" << upPFModel;
#endif

        if(upPassType && upPass1 && upPass2 && upPass3 && upValuePF && upCMDPF && upPFModel) {
            emit ptrThis->confirmNewPowerFactor(true);
        } else {
            emit ptrThis->confirmNewPowerFactor(false);

        }
        break;
    }

        default:
            break;
        }

}



 void InverterComm::handleMessageGrowatt(uint8_t lowReg, uint8_t bFuncCode, QByteArray mData, QByteArray recVexecute)
{
    (void)bFuncCode;
    (void)mData;
    (void)lowReg;

#if DEBUG
    qDebug() << " [GROWATT]Parse Geral da mensagem";
    qDebug() << " LOW REG " << lowReg << " Func " << bFuncCode;
    qDebug() << "mdata" << mData.data();
    qDebug() << "tamanho" << mData.length();

#endif
   if(!goodWe_GroWatt) {
    switch (bFuncCode) {
    case GROWATT_READ_HOLDING: {
#if DEBUG
        qDebug() << "\tParse Geral -> Parse Register HOLDING";
#endif

        handleReadHoldGrowatt(lowReg, mData);
        break;
    }
    case GROWATT_READ_INPUT: {
#if DEBUG
        qDebug() << "\tParse Geral -> Parse Read Register Input";
        qDebug() << "\tTAB INFO UPDATE";

#endif
        handleReadInputGrowatt(mData);
        break;
    }
    case GROWATT_PRESET_SINGLE_REG: {
#if DEBUG
        qDebug() << "\tParse Geral -> Parse Read Preset Reg command";
#endif
        handleExecuteGWTMessage(lowReg, recVexecute);
        //por enquanto apenas retorna
        break;
    }

    default:
        break;
    }
  }

}


bool InverterComm::bExecuteGwt(uint8_t low_reg)
{
    bool ret = false;
    QMutex tryWrite;
    static uint8_t abHIDBuffer[INVERTER_REG_GROWATT_BUFFER_SIZE];
#if DEBUG
    qDebug() << "[GROWATT] bExecuteGwt -  executando";
    qDebug() << "PARANDO TIMER";
#endif
    pPoolTimer->stop();
    tryWrite.lock();
    porta_serial->clear();
    porta_serial->clearError();
    firstUpdateOk = false;
    if ((this->bDevConnected) && (this->bDevId != 0)) {
#if DEBUG
    qDebug() << "[GROWATT] bExecuteGwt -  conectado";
    qDebug() << low_reg;
#endif
        int dSentBytes = 0;
        QByteArray mDataArray;

        // para mudanca de VAC High Low e time start
        if (low_reg == GROWATT_VAC_TIME_START_LO) {
#if DEBUG
    qDebug() << "\t\t Atualizando Vac - high low \n \t\t\t time reconnect and start";
#endif
            uint8_t delay_hi = 0x00, delay_lo = 0x00, hiVac_hi = 0x00,hiVac_lo = 0x00, loVac_hi = 0x00,loVac_lo = 0x00, reconnect_hi = 0x00, reconnect_lo = 0x00;
//          uint8_t hi_freq_lo = 0x00, hi_freq_hi = 0x00,lo_freq_lo = 0x00, lo_freq_hi = 0x00;
            uint16_t hiVac, loVac;
//          uint16_t loFac, hiFac;

            delay_lo = static_cast<uint8_t>(sInvCurrentConfig.wStartDelay & 0xFF);
            delay_hi = static_cast<uint8_t>((sInvCurrentConfig.wStartDelay >> 8) & 0xFF);
            reconnect_lo = static_cast<uint8_t>(sInvCurrentConfig.wReconnectTime & 0xFF);
            reconnect_hi = static_cast<uint8_t>((sInvCurrentConfig.wReconnectTime >> 8) & 0xFF);

            hiVac = sInvCurrentConfig.wMaxGridVoltage;
            loVac = sInvCurrentConfig.wMinGridVoltage;
            hiVac_lo = static_cast<uint8_t>(hiVac & 0xFF);
            hiVac_hi = static_cast<uint8_t>((hiVac >> 8) & 0xFF);
            loVac_lo = static_cast<uint8_t>(loVac & 0xFF);
            loVac_hi = static_cast<uint8_t>((loVac >> 8) & 0xFF);
//            hiFac = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;
//            hi_freq_lo = static_cast<uint8_t>(hiFac & 0xFF);
//            hi_freq_hi = static_cast<uint8_t>((hiFac >> 8) & 0xFF);
//            loFac = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
//            lo_freq_lo = static_cast<uint8_t>(loFac & 0xFF);
//            lo_freq_hi = static_cast<uint8_t>((loFac >> 8) & 0xFF);


#if DEBUG
    qDebug() << "bExecute -  executando -- envio ";
#endif
        memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
        porta_serial->clear(QSerialPort::Input);
        if (low_reg == GROWATT_VAC_TIME_START_LO) {
            bool check = false;
            int contLo = 0;


            // montagem e envio delay
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_VAC_TIME_START_HI, GROWATT_VAC_TIME_START_LO,delay_hi,delay_lo);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                check = false;
                contLo = 0;
                    do {
                    msleep(MSLEEP_TIME_TO_WRITE);
                    dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE ));
                    (void)dSentBytes; // To prevent warning when DEBUG is disable
                    check = readDevice();
                    if(!check) {
                        // falha ao atualizar
                        ret = false;
#if DEBUG
    qDebug() <<"DEU MERDA E NINGUEM VIU  [ DELAY ]" << contLo;
#endif
                    }
                    contLo++;
                    }while( !check && contLo < 3);
                    if(contLo >= 3 && check == false) {
                        emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
                       // return ret;
                    }
                    ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando Delay";
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
            // envio time reconnection 0x77
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RECONNECT_TIME_HI, GROWATT_RECONNECT_TIME_LO,reconnect_hi,reconnect_lo);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                check = false;
                contLo = 0;
                    do {
                    msleep(MSLEEP_TIME_TO_WRITE);
                    dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE ));
                    (void)dSentBytes; // To prevent warning when DEBUG is disable
                  //  porta_serial->waitForReadyRead(50);
                    check = readDevice();
                    if(!check) {
                        // falha ao atualizar
                        ret = false;
#if DEBUG
    qDebug() <<"DEU MERDA E NINGUEM VIU  [ reconnect time ]" << contLo;
#endif
                        //emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);


                    }
                    contLo++;
                    }while( !check && contLo < 4);
                    if(contLo >= 4 && check == false) {
                        emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
                        //return ret;
                    }
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando Tempo de Reconexão 0x77";
            qDebug() << hex << reconnect_hi << hex << reconnect_lo;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

            // envio high vac
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_VAC_HIGH_HI, GROWATT_VAC_HIGH_LO,hiVac_hi,hiVac_lo);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                check = false;
                contLo = 0;
                    do {
                        msleep(MSLEEP_TIME_TO_WRITE);
                        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE ));
                        (void)dSentBytes; // To prevent warning when DEBUG is disable
                       // porta_serial->waitForReadyRead(50);
                        check = readDevice();
                        if(!check ) {
                            // falha ao atualizar
                            ret = false;
    #if DEBUG
        qDebug() <<"DEU MERDA E NINGUEM VIU  [ high vac ]" << contLo;
    #endif
                            //emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);


                        }
                        contLo++;
                    }while( !check && contLo < 2);
                    if(contLo >= 2 && check == false) {
                        emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
                        return ret;
                    }
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando high Vac";
            qDebug() << hex << hiVac_hi << hex << hiVac_lo;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

            // envio low vac
            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_VAC_LOW_HI, GROWATT_VAC_LOW_LO,loVac_hi,loVac_lo);
            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
            check = false;
            contLo = 0;
                do {
                    msleep(MSLEEP_TIME_TO_WRITE);
                    dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE ));
                    (void)dSentBytes; // To prevent warning when DEBUG is disable
                 //   porta_serial->waitForReadyRead(50);
                    check = readDevice();
                    if(!check) {
                        // falha ao atualizar
                        ret = false;
    #if DEBUG
        qDebug() <<"DEU MERDA E NINGUEM VIU  [ LOW VAC  ]" << contLo;
    #endif


                    }
                    contLo++;
                   }while( !check && contLo < 8);
                if(contLo >= 8 && check == false) {
                    emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
                    return ret;
                }
                ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando low Vac";
            qDebug() << hex << loVac_hi << hex << loVac_lo;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
        }
        }
        // para sincronizar com tempo
        else if (low_reg == GROWATT_RTC_YEAR_LO) {
#if DEBUG
    qDebug() << "\t\t Atualizando RTC";
#endif
            uint16_t s_year = static_cast<uint16_t>(sInvCurrentRTCTime.bYear);
            uint8_t b_year_lo = static_cast<uint8_t>(s_year & 0xFF);
            uint8_t b_year_hi = static_cast<uint8_t>((s_year >> 8) & 0xFF);

            uint8_t s_month = static_cast<uint8_t>(sInvCurrentRTCTime.bMonth);
            uint8_t s_day = static_cast<uint8_t>(sInvCurrentRTCTime.bDay);
            uint8_t s_hour = static_cast<uint8_t>(sInvCurrentRTCTime.bHour);
            uint8_t s_minute = static_cast<uint8_t>(sInvCurrentRTCTime.bMinute);
            //uint8_t s_second = static_cast<uint8_t>(sInvCurrentRTCTime.bSecond);

            // envio ano
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RTC_YEAR_HI, GROWATT_RTC_YEAR_LO,b_year_hi,b_year_lo);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
                (void)dSentBytes; // To prevent warning when DEBUG is disable
                readDevice();
                ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando RTC tempo - ANO";
            qDebug() << hex << b_year_hi << hex << b_year_lo;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
            // envio mes
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RTC_MONTH_HI, GROWATT_RTC_MONTH_LO,0x00,s_month);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
                (void)dSentBytes; // To prevent warning when DEBUG is disable
                readDevice();
                ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando RTC tempo - Mes";
            qDebug() <<  "0x00" << hex << s_month;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

            // envio dia
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RTC_DAY_HI, GROWATT_RTC_DAY_LO,0x00,s_day);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
                (void)dSentBytes; // To prevent warning when DEBUG is disable
                readDevice();
                ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando RTC tempo - Dia";
            qDebug() <<  "0x00" << hex << s_day;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

            // envio horas
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RTC_HOUR_HI, GROWATT_RTC_HOUR_LO,0x00,s_hour);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
                (void)dSentBytes; // To prevent warning when DEBUG is disable
                readDevice();
                ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Atualizando RTC tempo - horas";
            qDebug() <<  "0x00" << hex << s_hour;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

            // envio minutos
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RTC_MINUTE_HI, GROWATT_RTC_MINUTE_LO,0x00,s_minute);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
                (void)dSentBytes; // To prevent warning when DEBUG is disable
                ret = true;
                readDevice();            }
#if DEBUG
            qDebug() << "[GWT]Atualizando RTC tempo - minutos";
            qDebug() <<  "0x00" << hex << s_minute;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

//            // envio segundo
//            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

//            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_RTC_SEC_HI, GROWATT_RTC_SEC_LO,0x00,s_second);

//            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
//                ret = false;
//            }
//            else {
//                if(usedGWTfunc){
//                    bGWTfunc = gtpcPacket->getLowAddr();
//                    usedGWTfunc = false;
//                }
//                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
//                porta_serial->waitForBytesWritten(1000);
//                (void)dSentBytes; // To prevent warning when DEBUG is disable
//                ret = true;
//            }
//#if DEBUG
//            qDebug() << "[GWT]Atualizando RTC tempo - segundos";
//            qDebug() <<  "0x00" << hex << s_second;
//            qDebug() << "Bytes written: " << dSentBytes;
//            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
//#endif

        }

        // setando novo valor de fator de potencia
        else if (low_reg == GROWATT_POWER_FACTOR_LO) {
#if DEBUG
    qDebug() << "[GROWATT] POWER FACTOR";
#endif
// para atualizar colocar senha em 88 - 8A (XX|XX|XX) unlock no 0x87
//  00 em 02
// pw factor em si
// 63
#if DEBUG
    qDebug() << "[GROWATT] POWER FACTOR - SpecPassword Unlock or set [0x87]";
#endif
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
    gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_PASSWORD_TYPE_HI, GROWATT_PASSWORD_TYPE_LO,GROWATT_PASSWORD_TYPE_VALUE,GROWATT_PASSWORD_TYPE_VALUE);

    if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
        ret = false;
    }
    else {
        if(usedGWTfunc){
            bGWTfunc = gtpcPacket->getLowAddr();
            usedGWTfunc = false;
        }
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
        (void)dSentBytes; // To prevent warning when DEBUG is disable
        ret = true;
        readDevice();
    }
#if DEBUG
    qDebug() << "[GWT]Atualizando POWER FACTOR specpassword unlock or set";
    qDebug() << "Bytes written: " << dSentBytes;
    qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
// password type OK
// escrevendo XX;
    for (int i = 0; i < 3; i++)  {
#if DEBUG
    qDebug() << "[GROWATT] POWER FACTOR - SpecPassword Unlock or set [0x88 - 0x8A]";
#endif
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

    gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_PASSWORD_HI,static_cast<uint8_t>(GROWATT_PASSWORD_LO + i),GROWATT_PASSWORD_VALUE,GROWATT_PASSWORD_VALUE);

    if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
        ret = false;
    }
    else {
        if(usedGWTfunc){
            bGWTfunc = gtpcPacket->getLowAddr();
            usedGWTfunc = false;
        }
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
        (void)dSentBytes; // To prevent warning when DEBUG is disable
        ret = true;
        readDevice();
    }
#if DEBUG
    qDebug() << "[GWT]Atualizando POWER FACTOR specpassword set [0x88 + " << i <<"]";
    qDebug() << GROWATT_PASSWORD_VALUE;
    qDebug() << "Bytes written: " << dSentBytes;
    qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

    }
// ESCRITO XX \ XX \ XX
// ESCREVENDO 0X02
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

    gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_POWER_FACTOR_CMD_HI, GROWATT_POWER_FACTOR_CMD_LO,GROWATT_POWER_FACTOR_CMD_OFFSET_HI,0x00);


    if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
        ret = false;
    }
    else {
        if(usedGWTfunc){
            bGWTfunc = gtpcPacket->getLowAddr();
            usedGWTfunc = false;
        }
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
        (void)dSentBytes; // To prevent warning when DEBUG is disable
        ret = true;
        readDevice();
    }
#if DEBUG
    qDebug() << "[GWT]Atualizando CMD MEMORY STATE POWER FACTOR";
    qDebug() << GROWATT_WRITE_PF_MODEL_CMD;
    qDebug() << "Bytes written: " << dSentBytes;
    qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

        gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_POWER_FACTOR_HI, GROWATT_POWER_FACTOR_LO,sGWTpowFactor.bPowerFactorHigh,sGWTpowFactor.bPowerFactorLow);

        if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
            ret = false;
        }
        else {
            if(usedGWTfunc){
                bGWTfunc = gtpcPacket->getLowAddr();
                usedGWTfunc = false;
            }
            dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
            ret = true;
            readDevice();
        }
#if DEBUG
            qDebug() << "[GWT]Atualizando POWER FACTOR - VALUE";
            qDebug() << hex << sGWTpowFactor.bPowerFactorHigh << hex << sGWTpowFactor.bPowerFactorLow;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));

            gtpcPacket = new ProtocolGrowatt(GwtMasterAddr, GROWATT_PRESET_SINGLE_REG,GROWATT_PF_MODE_HI, GROWATT_PF_MODEL_LO,0x00,GROWATT_WRITE_PF_MODEL_CMD);

            if (!gtpcPacket->toBufferGrowatt(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWTfunc){
                    bGWTfunc = gtpcPacket->getLowAddr();
                    usedGWTfunc = false;
                }
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), sizeof(abHIDBuffer)));
                (void)dSentBytes; // To prevent warning when DEBUG is disable
                ret = true;
            }
#if DEBUG
            qDebug() << "[GWT]Setando PF MODEL 0x63 -- 99 ";
            qDebug() << GROWATT_WRITE_PF_MODEL_CMD;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
            readDevice();

        }

    }
    msleep(MSLEEP_TIME_TO_WRITE);
    tryWrite.unlock();
    pPoolTimer->start();
    firstUpdateOk = true;
    return ret;
}
#if LOGG
void myMessageOutput(QtMsgType type, const QMessageLogContext &, const QString &msg)
{
    QString texto, arquivo, fline;
    QString horaCerta = QDateTime::currentDateTime().toString("dd-MM-yyyy HH:mm:ss");

    if(firstLineLog){
        fline = QString("\n\n\n\n\n"
                        "\t\t---------------------------------------------------------\n"
                        "\t\t---------= COMEÇO DA EXECUÇÃO NOVA DO PROGRAMA =---------\n"
                        "\t\t---------------------------------------------------------\n"
                        "\n");
    }

    switch (type) {
    case QtDebugMsg:
        texto = QString("[%1]Debug: %2\n").arg(horaCerta,msg);
        cDebug++;
        break;
    case QtInfoMsg:
        texto = QString("[%1]Info: %2 \n").arg(horaCerta,msg);
        cInfo++;
        break;
    case QtWarningMsg:
        texto = QString("[%1]Warning: %s\n").arg(horaCerta,msg);
        cWarning++;
        break;
    case QtCriticalMsg:
        texto = QString("[%1]Critical: %2\n").arg(horaCerta,msg);
        cCritical++;
        break;
    case QtFatalMsg:
        texto = QString("[%1]Fatal: %2\n").arg(horaCerta,msg);
        cError++;
        break;

    }
    arquivo = QString("NHS_Tool.log");
    QFile outFile(arquivo);
    outFile.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream ts(&outFile);
    if(firstLineLog){
        ts << fline << endl;
        firstLineLog = false;
    }
    ts << texto << endl;

}
#endif

bool InverterComm::bReadGWe_modbus(uint8_t functCode) {

    bool ret = true;
#if DEBUG
            qDebug() << "[GOODWE MODBUS] ... READ REG";
#endif
    static uint8_t sendBuffer[INVERTER_REG_GROWATT_BUFFER_SIZE];
    if ((this->bDevConnected) && (this->bDevId != 0)) {
        memset(sendBuffer, 0x00, sizeof(sendBuffer));

        switch (functCode) {
        case GOODWE_RTC_YEAR_MONTH: {
#if DEBUG
    qDebug() << "\t RTC";
#endif
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_READ_FUNCTION,GOODWE_RTC_LO,GOODWE_RTC_YEAR_MONTH,GOODWE_RTC_OFFSET_HI,GOODWE_RTC_OFFSET_LO);
            break;
        }
        case GOODWE_RECONNECT_TIME: {
#if DEBUG
    qDebug() << "\t Inverter Config";
#endif
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_READ_FUNCTION,GOODWE_RTC_LO,GOODWE_RECONNECT_TIME,GOODWE_INV_CONFIG_OFFSET_HI,GOODWE_INV_CONFIG_OFFSET_LO);
            break;
        }
        case GOODWE_ERROR_CODE_LO: {
#if DEBUG
    qDebug() << "\t Inverter All setings";
#endif
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_READ_FUNCTION,GOODWE_ERROR_CODE_HI,GOODWE_ERROR_CODE_LO,GOODWE_ALL_SET_OFFSET_HI,GOODWE_ALL_SET_OFFSET_LO);
            break;
        }
        case GOODWE_ABOUT_SERIAL: {
#if DEBUG
    qDebug() << "\t Inverter about config";
#endif
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_READ_FUNCTION,GOODWE_SERIAL_NUMBER_HI,GOODWE_SERIAL_NUMBER_LO,GOODWE_ABOUT_SERIAL_OFFSET_HI,GOODWE_ABOUT_SERIAL_OFFSET_LO);
            break;
        }
        case GOODWE_ABOUT_MODELO: {
#if DEBUG
    qDebug() << "\t Inverter about config";
#endif
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_READ_FUNCTION,GOODWE_MODEL_NAME_HI,GOODWE_MODEL_NAME_LO,GOODWE_ABOUT_MODELO_OFFSET_HI,GOODWE_ABOUT_MODELO_OFFSET_LO);
            break;
        }
        case GOODWE_POWER_FACTOR: {
#if DEBUG
    qDebug() << "\t Inverter POWER FACTOR READ";
#endif
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_READ_FUNCTION,GOODWE_FEEDING_PW_HI,GOODWE_FEEDING_PW_LO,GOODWE_LOW_FEED_VOLT_HI_LO,GOODWE_LOW_FEED_VOLT_OFFSET_LO);
            break;
        }

        default : {
            return false;
        }

        }

        if (!gtmodBusPacket->toBufferGoodWe(&sendBuffer[0])) {
            ret = false;
#if DEBUG
            qDebug() << "Error to parse message...";
#endif
        }
        else {
            if(usedGWeModfunc){
                bGWEfunc = static_cast<uint16_t>((gtmodBusPacket->getHiAddr() << 8) | gtmodBusPacket->getLowAddr());
                usedGWeModfunc = false;
            }
            int dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(sendBuffer), INVERTER_REG_GROWATT_BUFFER_SIZE ));
            (void)dSentBytes; // To prevent warning when DEBUG is disable
#if DEBUG
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (sendBuffer);
#endif

        }

    }
    return ret;

}

bool InverterComm::bExecuteGW_modbus(uint16_t low_reg) {
#if DEBUG
            qDebug() << "\t-bExecuteGW_modbus";
#endif
            bool ret = false;
            bool check = false;
            QMutex tryWrite;
            static uint8_t abHIDBuffer[INVERTER_REG_GOODWE_WRITE];
            static uint8_t abHIDBufferRTC[INVERTER_REG_GOODWE_WRITE_RTC];
        #if DEBUG
            qDebug() << "[GOODWE ModBus] bExecuteGW_modbus -  executando";
            qDebug() << "PARANDO TIMER";
        #endif
            pPoolTimer->stop();
            tryWrite.lock();
            porta_serial->clear();
            porta_serial->clearError();
            firstUpdateOk = false;
            if ((this->bDevConnected) && (this->bDevId != 0)) {
        #if DEBUG
            qDebug() << "[GOODWE ModBus] bExecuteGW_modbus -  conectado";
            qDebug() << low_reg;
        #endif
                int dSentBytes = 0;
                QByteArray mDataArray;
    switch (low_reg) {
    case GOODWE_RECONNECT_TIME_WRITE : {
#if DEBUG
    qDebug() << "\t\t Atualizando Vac - high low \n \t\t\t time reconnect and start";
#endif
            uint8_t delay_hi = 0x00, delay_lo = 0x00, hiVac_hi = 0x00,hiVac_lo = 0x00, loVac_hi = 0x00,loVac_lo = 0x00;
//          uint8_t hi_freq_lo = 0x00, hi_freq_hi = 0x00,lo_freq_lo = 0x00, lo_freq_hi = 0x00;
            uint16_t hiVac, loVac;
//          uint16_t loFac, hiFac;

            delay_lo = static_cast<uint8_t>(sInvCurrentConfig.wStartDelay & 0xFF);
            delay_hi = static_cast<uint8_t>((sInvCurrentConfig.wStartDelay >> 8) & 0xFF);

            hiVac = sInvCurrentConfig.wMaxGridVoltage;
            loVac = sInvCurrentConfig.wMinGridVoltage;
            hiVac_lo = static_cast<uint8_t>(hiVac & 0xFF);
            hiVac_hi = static_cast<uint8_t>((hiVac >> 8) & 0xFF);
            loVac_lo = static_cast<uint8_t>(loVac & 0xFF);
            loVac_hi = static_cast<uint8_t>((loVac >> 8) & 0xFF);
//            hiFac = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;
//            hi_freq_lo = static_cast<uint8_t>(hiFac & 0xFF);
//            hi_freq_hi = static_cast<uint8_t>((hiFac >> 8) & 0xFF);
//            loFac = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
//            lo_freq_lo = static_cast<uint8_t>(loFac & 0xFF);
//            lo_freq_hi = static_cast<uint8_t>((loFac >> 8) & 0xFF);


#if DEBUG
    qDebug() << "bExecute -  executando -- envio ";
#endif
    check = false;

    // montagem e envio delay
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
    gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_WRITE_FUNCTION,GOODWE_HIGH_BYTE_00, GOODWE_RECONNECT_TIME,GOODWE_HIGH_BYTE_AMOUNT,GOODWE_LOW_BYTE_AMOUNT,GOODWE_AMOUNT_BYTES,delay_hi,delay_lo);

    if (!gtmodBusPacket->toBufferGoodWe(&abHIDBuffer[0])) {
        ret = false;
    }
    else {
        if(usedGWeModfunc){
            bGWEfunc = static_cast<uint16_t>((gtmodBusPacket->getHiAddr() << 8) | gtmodBusPacket->getLowAddr());
            usedGWeModfunc = false;
        }
        check = false;
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GOODWE_WRITE ));
        msleep(MSLEEP_TIME_TO_WRITE);
        (void)dSentBytes; // To prevent warning when DEBUG is disable
        //porta_serial->waitForReadyRead(50);
        check = readDevice();
        if(check == false) {
           emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
           // return ret;
       }
       ret = true;
    }
#if DEBUG
    qDebug() << "[GWE MODBUS]Atualizando Delay";
    qDebug() << "Bytes written: " << dSentBytes;
    qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
    // envio high vac
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
    gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_WRITE_FUNCTION,GOODWE_HIGH_BYTE_00, GOODWE_HIGH_GRID_VOLTAGE,GOODWE_HIGH_BYTE_AMOUNT,GOODWE_LOW_BYTE_AMOUNT,GOODWE_AMOUNT_BYTES,hiVac_hi,hiVac_lo);

    if (!gtmodBusPacket->toBufferGoodWe(&abHIDBuffer[0])) {
        ret = false;
    }
    else {
        if(usedGWeModfunc){
            bGWEfunc = static_cast<uint16_t>((gtmodBusPacket->getHiAddr() << 8) | gtmodBusPacket->getLowAddr());
            usedGWeModfunc = false;
        }
        check = false;
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GOODWE_WRITE ));
        msleep(MSLEEP_TIME_TO_WRITE);
        (void)dSentBytes; // To prevent warning when DEBUG is disable
      //  porta_serial->waitForReadyRead(50);
        check = readDevice();
        if(check == false) {
           emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
           // return ret;
       }
       ret = true;
    }



#if DEBUG
    qDebug() << "[GOODWE ModBus]Atualizando high Vac";
    qDebug() << hex << hiVac_hi << hex << hiVac_lo;
    qDebug() << "Bytes written: " << dSentBytes;
    qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

    // envio low vac
    memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
    gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_WRITE_FUNCTION,GOODWE_HIGH_BYTE_00,GOODWE_LOW_GRID_VOLTAGE, GOODWE_HIGH_BYTE_AMOUNT,GOODWE_LOW_BYTE_AMOUNT,GOODWE_AMOUNT_BYTES,loVac_hi,loVac_lo);

    if (!gtmodBusPacket->toBufferGoodWe(&abHIDBuffer[0])) {
        ret = false;
    }
    else {
        if(usedGWeModfunc){
            bGWEfunc = static_cast<uint16_t>((gtmodBusPacket->getHiAddr() << 8) | gtmodBusPacket->getLowAddr());
            usedGWeModfunc = false;
        }
        check = false;
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GOODWE_WRITE ));
        msleep(MSLEEP_TIME_TO_WRITE);
        (void)dSentBytes; // To prevent warning when DEBUG is disable
       // porta_serial->waitForReadyRead(50);
        check = readDevice();
        if(check == false) {
           emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
           // return ret;
       }
       ret = true;
    }
#if DEBUG
    qDebug() << "[GOODWE ModBus]Atualizando low Vac";
    qDebug() << hex << loVac_hi << hex << loVac_lo;
    qDebug() << "Bytes written: " << dSentBytes;
    qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif

    } break;

    case GOODWE_RTC : {
#if DEBUG
    qDebug() << "\t\t Atualizando RTC";
#endif
            uint16_t s_year = static_cast<uint16_t>(sInvCurrentRTCTime.bYear);
            uint8_t b_year_lo = static_cast<uint8_t>(s_year & 0xFF);

            uint8_t s_month = static_cast<uint8_t>(sInvCurrentRTCTime.bMonth);
            uint8_t s_day = static_cast<uint8_t>(sInvCurrentRTCTime.bDay);
            uint8_t s_hour = static_cast<uint8_t>(sInvCurrentRTCTime.bHour);
            uint8_t s_minute = static_cast<uint8_t>(sInvCurrentRTCTime.bMinute);
            uint8_t s_second = static_cast<uint8_t>(sInvCurrentRTCTime.bSecond);

    check = false;

    // montagem e envio ano, mes, dia, hora, minutos, segundos
    memset(abHIDBufferRTC, 0x00, sizeof(abHIDBufferRTC));
    gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_WRITE_FUNCTION,GOODWE_HIGH_BYTE_00, GOODWE_RTC_YEAR_MONTH,GOODWE_HIGH_BYTE_AMOUNT,GOODWE_LOW_BYTE_AMOUNT_RTC,GOODWE_AMOUNT_BYTES_RTC,b_year_lo, s_month,s_day,s_hour, s_minute, s_second);

    if (!gtmodBusPacket->toBufferGoodWe(&abHIDBufferRTC[0])) {
        ret = false;
    }
    else {
        if(usedGWeModfunc){
            bGWEfunc = static_cast<uint16_t>((gtmodBusPacket->getHiAddr() << 8) | gtmodBusPacket->getLowAddr());
            usedGWeModfunc = false;
        }
        check = false;
        dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBufferRTC), INVERTER_REG_GOODWE_WRITE_RTC));
        msleep(MSLEEP_TIME_TO_WRITE);
        (void)dSentBytes; // To prevent warning when DEBUG is disable
      //  porta_serial->waitForReadyRead(50);
        check = readDevice();
        if(check == false) {
           emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
           // return ret;
       }
       ret = true;
    }
#if DEBUG
            qDebug() << "[GWT]Atualizando RTC tempo - ANO - mes";
            qDebug() << s_day<<"/"<< s_month << "/"  << b_year_lo;
            qDebug() << s_hour<<":"<< s_minute << ":" << s_second;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBufferRTC);
#endif
    } break;
    case GOODWE_RANGE_REAC_POWER: {
#if DEBUG
            qDebug() << "[GWE MODBUS]Atualizando power factor";
#endif
            check = false;

            // montagem e envio power factor
            memset(abHIDBuffer, 0x00, sizeof(abHIDBuffer));
            gtmodBusPacket = new ProtocolGoodWe(ModBusMasterAddr, GOODWE_WRITE_FUNCTION,GOODWE_RANGE_REAC_POWER_ADJUST_HI, GOODWE_RANGE_REAC_POWER_ADJUST_LO,GOODWE_HIGH_BYTE_AMOUNT,GOODWE_LOW_BYTE_AMOUNT,GOODWE_AMOUNT_BYTES,sGWEpowFactor.bGWEPowerFactorHigh,sGWEpowFactor.bGWEPowerFactorLow);

            if (!gtmodBusPacket->toBufferGoodWe(&abHIDBuffer[0])) {
                ret = false;
            }
            else {
                if(usedGWeModfunc){
                    bGWEfunc = static_cast<uint16_t>((gtmodBusPacket->getHiAddr() << 8) | gtmodBusPacket->getLowAddr());
                    usedGWeModfunc = false;
                }
                check = false;
                dSentBytes = sendData(QByteArray(reinterpret_cast<const char*>(abHIDBuffer), INVERTER_REG_GOODWE_WRITE ));
                msleep(100);
                (void)dSentBytes; // To prevent warning when DEBUG is disable
            //    porta_serial->waitForReadyRead(50);
                check = readDevice();
                if(check == false) {
                   emit ptrThis->confirmNewSettings(INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL);
                   // return ret;
               }
               ret = true;
            }
#if DEBUG
            qDebug() << "[GWE]Atualizando power factor";
            qDebug() << "power facor high " << hex << sGWEpowFactor.bGWEPowerFactorHigh << " power facor low "<<hex << sGWEpowFactor.bGWEPowerFactorLow;
            qDebug() << "Bytes written: " << dSentBytes;
            qDebug() << "Tamanho: " << sizeof (abHIDBuffer);
#endif
    } break;

    default:
        ret = false;

    }

  }

    msleep(MSLEEP_TIME_TO_WRITE);
    tryWrite.unlock();
    pPoolTimer->start();
    firstUpdateOk = true;
    return ret;
}

void handleReadHoldModBus(uint16_t lowReg, QByteArray mData) {
#if DEBUG
    qDebug() << "Parse da mensagem de read modbus goodwe:";
    qDebug() << "[ModBus]func" << lowReg;
    if(!mData.isEmpty()){
        qDebug() << "tamanho" << mData.length();
        qDebug() << "dado:" << mData.toHex();
    }
#endif


        switch (lowReg) {
        case GOODWE_SERIAL_NUMBER: {
#if DEBUG
            qDebug() << "\tModBus Response About Info (SERIE)";
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == GOODWE_RESPONDE_SERIAL_LENGHT) {
                memcpy(sInvIDInfoData.abSerialNumber, &pbRecvData[0], GOODWE_RESPONDE_SERIAL_LENGHT);
#if DEBUG
                qDebug() << "Atualizando about";
#endif
                emit ptrThis->updateAboutTab(sInvIDInfoData);
            }
            break;
        }
        case GOODWE_MODEL_NAME: {
#if DEBUG
            qDebug() << "\t[ModBus] Response About Info (model NAME)";
#endif
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == GOODWE_RESPONSE_MODEL_LENGHT) {
                memcpy(sInvIDInfoData.abModelName, &pbRecvData[0], GOODWE_RESPONSE_MODEL_LENGHT);
#if DEBUG
                qDebug() << "\t[ModBus]Atualizando about";
#endif
                emit ptrThis->updateAboutTab(sInvIDInfoData);
            }
            break;
        }
        case GOODWE_RTC: {
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
#if DEBUG
                qDebug() << "\t[ModBus]ModBus -- resposta RTC -- tamanho do pacote" << mData.length();
#endif
            if (mData.length() == GOODWE_RESPONSE_RTC_LENGHT) {
                sInvCurrentRTCTime.bYear = pbRecvData[GWE_INV_RTC_TIME_INFO_YEAR_OFFSET];
                sInvCurrentRTCTime.bMonth = pbRecvData[GWE_INV_RTC_TIME_INFO_MONTH_OFFSET];
                sInvCurrentRTCTime.bDay = pbRecvData[GWE_INV_RTC_TIME_INFO_DAY_OFFSET];
                sInvCurrentRTCTime.bHour = pbRecvData[GWE_INV_RTC_TIME_INFO_HOUR_OFFSET];
                sInvCurrentRTCTime.bMinute = pbRecvData[GWE_INV_RTC_TIME_INFO_MINUTE_OFFSET];
                sInvCurrentRTCTime.bSecond = pbRecvData[GWE_INV_RTC_TIME_INFO_SECOND_OFFSET];
#if DEBUG
                qDebug() << "\t[ModBus]ATUALIZANDO \t[ModBus]-- RTC info ";
                qDebug() << sInvCurrentRTCTime.bDay  << "/" << sInvCurrentRTCTime.bMonth << "/" << sInvCurrentRTCTime.bYear;
                qDebug() << sInvCurrentRTCTime.bHour << ":" << sInvCurrentRTCTime.bMinute << ":" << sInvCurrentRTCTime.bSecond ;
#endif

                emit ptrThis->updateCurrentRTCTime(sInvCurrentRTCTime, 2000U);
            }
         break;
        }
        case GOODWE_RECONNECT_TIME: {
#if DEBUG
                qDebug() << "[ModBus]resposta settings -- tamanho do pacote" << mData.length();
#endif

            uint8_t* pwRecvData = reinterpret_cast<uint8_t*>(mData.data());

            if (mData.length() == GOODWE_CONFIG_LENGHT) {
                sInvCurrentConfigUI.wStartDelay = static_cast<uint16_t>((pwRecvData[GOODWE_RESPONSE_DELAY_TIME] << 8) | pwRecvData[GOODWE_RESPONSE_DELAY_TIME + 1]);
                sInvCurrentConfigUI.eOutVoltage = getOutputVoltageIndex(pwRecvData);

                emit ptrThis->updateConfigTab(sInvCurrentConfigUI);
#if DEBUG
                qDebug() << "[ModBus]ATUALIZANDO -- setting info ";
                qDebug() << "Start Delay:" << sInvCurrentConfigUI.wStartDelay;
                qDebug() << "eOutVoltage:" << sInvCurrentConfigUI.eOutVoltage;
#endif
            }
         break;
        }
        case GOODWE_ERROR_CODE: {
#if DEBUG
            qDebug() << "\t[ModBus]\tResponse Running Info";
#endif
                uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());

                if ((mData.length() == GOODWE_RESPONSE_INFO_LENGHT)) {
#if DEBUG
    qDebug() << "\t[ModBus]Tamanho do pacote OK";
#endif
    sInvRunningInfoData.fPV1Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_PV_VOLTAGE_1] << 8) | pbRecvData[GOODWE_RESPONSE_PV_VOLTAGE_1 + 1])))) * 0.1f;
    sInvRunningInfoData.fPV1Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_PV_CURRENT_1] << 8) | pbRecvData[GOODWE_RESPONSE_PV_CURRENT_1 + 1])))) * 0.1f;
    sInvRunningInfoData.fPV2Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_PV_VOLTAGE_2] << 8) | GOODWE_RESPONSE_PV_VOLTAGE_2)))) * 0.1f;
    sInvRunningInfoData.fPV2Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_PV_CURRENT_2] << 8) | pbRecvData[GOODWE_RESPONSE_PV_CURRENT_2 + 1])))) * 0.1f;
    sInvRunningInfoData.fPhaseL1Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_VOLTAGE_PHASE1] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_VOLTAGE_PHASE1 + 1])))) * 0.1f;
    sInvRunningInfoData.fPhaseL1Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_CURRENT_PHASE1] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_CURRENT_PHASE1 + 1])))) * 0.1f;
    sInvRunningInfoData.fPhaseL2Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_VOLTAGE_PHASE2] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_VOLTAGE_PHASE2 + 1])))) * 0.1f;
    sInvRunningInfoData.fPhaseL2Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_CURRENT_PHASE2] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_CURRENT_PHASE2 + 1])))) * 0.1f;
    sInvRunningInfoData.fPhaseL3Voltage = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_VOLTAGE_PHASE3] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_VOLTAGE_PHASE3 + 1])))) * 0.1f;
    sInvRunningInfoData.fPhaseL3Current = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_CURRENT_PHASE3] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_CURRENT_PHASE3 + 1])))) * 0.1f;


    sInvRunningInfoData.wLowByteFeedingPW = static_cast<uint16_t>(((pbRecvData[GOODWE_RESPONSE_LOW_BYTE_FEEDING] << 8) | pbRecvData[ GOODWE_RESPONSE_LOW_BYTE_FEEDING + 1]));
    sInvRunningInfoData.bPowerFactor = static_cast<uint8_t>(((sInvRunningInfoData.wLowByteFeedingPW / (sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current)) * 100));


    sInvRunningInfoData.fPhaseL1Frequency = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_FREQUENCY_PHASE1] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_FREQUENCY_PHASE1 + 1])))) * 0.01f;
    sInvRunningInfoData.fPhaseL2Frequency = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_FREQUENCY_PHASE2] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_FREQUENCY_PHASE2 + 1])))) * 0.01f;
    sInvRunningInfoData.fPhaseL3Frequency = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_GRID_FREQUENCY_PHASE3] << 8) | pbRecvData[GOODWE_RESPONSE_GRID_FREQUENCY_PHASE3 + 1])))) * 0.01f;

    sInvRunningInfoData.eInvWorkMode = static_cast<eInverterWorkMode>(static_cast<uint32_t>(((pbRecvData[GOODWE_RESPONSE_STATUS] << 8) | pbRecvData[GOODWE_RESPONSE_STATUS + 1])));

    sInvRunningInfoData.fInternalTemp = static_cast<float>(static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_TEMPERATURE] << 8) | pbRecvData[GOODWE_RESPONSE_TEMPERATURE + 1])))) * 0.1f;

    sInvRunningInfoData.dErrorMessage = static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_ERROR_CODE] << 8) | pbRecvData[GOODWE_RESPONSE_ERROR_CODE + 1]) << 16));
    sInvRunningInfoData.dErrorMessage |= static_cast<uint32_t>(((pbRecvData[GOODWE_RESPONSE_ERROR_CODE+2] << 8) | pbRecvData[GOODWE_RESPONSE_ERROR_CODE + 3]));

    sInvRunningInfoData.fTotalEnergy2Grid = static_cast<float>((((pbRecvData[GOODWE_RESPONSE_ENERGY_TOTAL] << 8) | pbRecvData[GOODWE_RESPONSE_ENERGY_TOTAL + 1]) << 16))*0.1f;
    sInvRunningInfoData.fTotalEnergy2Grid = static_cast<float>((((pbRecvData[GOODWE_RESPONSE_ENERGY_TOTAL+2] << 8) | pbRecvData[GOODWE_RESPONSE_ENERGY_TOTAL + 3])))*0.1f;

    sInvRunningInfoData.dTotalFeedHours = static_cast<uint32_t>((((pbRecvData[GOODWE_RESPONSE_HOUR_TOTAL] << 8) | pbRecvData[GOODWE_RESPONSE_HOUR_TOTAL + 1]) << 16));
    sInvRunningInfoData.dTotalFeedHours |= static_cast<uint32_t>(((pbRecvData[GOODWE_RESPONSE_HOUR_TOTAL+2] << 8) | pbRecvData[GOODWE_RESPONSE_HOUR_TOTAL + 3]));
#if DEBUG
    qDebug() << "\t[ModBus]ATUALIZANDO";

    qDebug() << "PV1 Tensaão:" << sInvRunningInfoData.fPV1Voltage;
    qDebug() << "PV1 Corrente:" << sInvRunningInfoData.fPV1Current;
    qDebug() << "PV2 Tensaão:" << sInvRunningInfoData.fPV2Voltage;
    qDebug() << "PV2 Corrente:" << sInvRunningInfoData.fPV2Current;
    qDebug() << "Fase L1 Tensão:" << sInvRunningInfoData.fPhaseL1Voltage;
    qDebug() << "Fase L2 Tensão:" << sInvRunningInfoData.fPhaseL2Voltage;
    qDebug() << "Fase L3 Tensão:" << sInvRunningInfoData.fPhaseL3Voltage;
    qDebug() << "Fase L1 Corrente:" << sInvRunningInfoData.fPhaseL1Current;
    qDebug() << "Fase L2 Corrente:" << sInvRunningInfoData.fPhaseL2Current;
    qDebug() << "Fase L3 Corrente:" << sInvRunningInfoData.fPhaseL3Current;
    qDebug() << "Energia Status Rede:" << sInvRunningInfoData.fTotalEnergy2Grid;
    qDebug() << "Tempo Status Rede:" << sInvRunningInfoData.dTotalFeedHours;
    qDebug() << "Low byte feeding" << sInvRunningInfoData.wLowByteFeedingPW;
    qDebug() << "Power factor" << sInvRunningInfoData.bPowerFactor;
#endif

    emit ptrThis->updateInfoTab(sInvRunningInfoData);
#if DEBUG
    qDebug() << " ------ SUPOSTAMENTE ATUALIZANDO -------";
#endif

}
         break;
        }
        case GOODWE_FEEDING_PW: {
#if DEBUG
                qDebug() << "[ModBus]resposta power factor" << mData.length();
#endif
            float pfaux;
            uint8_t* pbRecvData = reinterpret_cast<uint8_t*>(mData.data());
            sInvRunningInfoData.wLowByteFeedingPW = static_cast<uint16_t>(((pbRecvData[0] << 8) | pbRecvData[1]));
            if( (sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current) == 0.0f) {
                pfaux = 0.0f;
            } else {
                pfaux = (sInvRunningInfoData.wLowByteFeedingPW / (sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current));

            }
            sInvRunningInfoData.bPowerFactor = static_cast<uint8_t>(((sInvRunningInfoData.wLowByteFeedingPW / (sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current)) * 100));

            if (mData.length() == 0x02) {

                emit ptrThis->updateInfoTab(sInvRunningInfoData);
#if DEBUG
    qDebug() << "\t Respostas power factor [ModBus]ATUALIZANDO";
    qDebug() << "lowByteFeedingPW:" << sInvRunningInfoData.wLowByteFeedingPW;
    qDebug() << "Multiplicacao" <<(sInvRunningInfoData.fPhaseL1Voltage * sInvRunningInfoData.fPhaseL1Current);
    qDebug() << "Power factor" << sInvRunningInfoData.bPowerFactor;
    qDebug() << "Power factor float" << pfaux;
#endif
            }
         break;
        }

        }
}

void handleWriteGoodWeModBus(uint16_t lowReg, QByteArray mData) {
    (void)mData;
    // depois tem que arrumar pois neste caso entra 3x e 6x na outra
#if DEBUG
    if(!mData.isEmpty())
        qDebug() << "-- execute -- " << mData.toHex();
    qDebug() << "lowReg" << lowReg;
#endif
    switch (lowReg) {

        case GOODWE_RECONNECT_TIME_WRITE: {
        if(valide_GWE_func_message(mData,lowReg)) {
            timeStart = true;
        }
            break;
        }
        case GOODWE_HIGH_GRID_VOLTAGE_W:{
            if(valide_GWE_func_message(mData,lowReg)) {
                vacHigh = true;
            }
            break;
    }
        case GOODWE_LOW_GRID_VOLTAGE_W: {
#if DEBUG
                qDebug() << "[GoodWe ModBus] Mensagem de VAC - high confirm" << timeStart << valLow << vacHigh;
#endif
        if(valide_GWE_func_message(mData,lowReg)) {
            valLow = true;
#if DEBUG
                qDebug() << "[GoodWe ModBus]Mensagem de VAC - high confirm" << timeStart << valLow << vacHigh;
#endif
            if(vacHigh && valLow && timeStart){
                emit ptrThis->confirmNewSettings(INV_SUCESS_WRITE_RETURN);
//                ptrThis->getInverterConfig();

            } else {
                emit ptrThis->confirmNewSettings(false);

            }
        }

            break;
    }


    case GOODWE_RTC: {
    if(valide_GWE_func_message(mData,lowReg)) {
        upyear = true; upmes = true;
        emit ptrThis->confirmNewRTCTime(INV_SUCESS_WRITE_RETURN);
    } else {
        emit ptrThis->confirmNewRTCTime(false);
    }

        break;
    }
    case GOODWE_RANGE_REAC_POWER: {
        if(valide_GWE_func_message(mData,lowReg)) {
            emit ptrThis->confirmNewPowerFactor(true);
        } else {
            emit ptrThis->confirmNewPowerFactor(false);
}
        
        break;
    }

    default:
        break;
    }

}

void InverterComm::handleMessageGoodWeModbus(uint16_t lowReg, uint8_t bFuncCode, QByteArray mData, QByteArray recVexecute) {
    (void)bFuncCode;
    (void)mData;
    (void)lowReg;
    (void) recVexecute;

#if DEBUG
    qDebug() << " [MODBUS GOODWE]Parse Geral da mensagem";
    qDebug() << " LOW REG " << lowReg << " Func " << bFuncCode;
    qDebug() << "mdata" << mData.data();
    qDebug() << "tamanho" << mData.length();
#endif

    if(goodWe_GroWatt && modbus_goodWe == false) {
        switch(bFuncCode) {
   case GOODWE_READ_FUNCTION: {
    #if DEBUG
            qDebug() << "\tParse Geral -> Parse Register READ modbus";
    #endif

            handleReadHoldModBus(lowReg, mData);
            break;
        }
        case GOODWE_WRITE_FUNCTION: {
         #if DEBUG
                 qDebug() << "\tParse Geral -> Parse Register write 0x10 modbus";
         #endif

                handleWriteGoodWeModBus(lowReg, recVexecute);
                 break;
             }
        case  GOODWE_NOT_GOOD_FUNCTION:
        case GOODWE_FAULTY_FUNCTION: {
         #if DEBUG
                 qDebug() << "\tParse Geral -> Parse Register erro msg 0x90 ou 0x83 modbus";
         #endif
                 emit ptrThis->updateStatusBar(QString(INV_WRITE_OR_READ_ERRO_GOODWE));

                 break;
             }

        }
    }
     usedGWeModfunc = true;

}

void InverterComm::statsFinal() {
#if DEBUG
    qDebug() << "\n\t------Estatística Finais do log------\n\tContadores :\n \tError = "
             << cError <<"\tInfo = " << cInfo << "\n \tWarning = " << cWarning <<"\tDebug = "
             << cDebug << "\tCritico = " << cCritical << "\n----------------------------------------------------";
#endif
}
