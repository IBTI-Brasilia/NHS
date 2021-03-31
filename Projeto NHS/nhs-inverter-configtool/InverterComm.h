#pragma once

#include <QThread>
#include <QDebug>
#include <QTimer>
#include <QMutex>
#include <QString>
#include <QtEndian>
#include <QDateTime>
#include <math.h>
#include "ProtocolPacket.h"
#include "utils.h"
#include "ProtocolGoodWe.h"
#include "ProtocolGrowatt.h"
//#include "qhidapi/qhidapi.h"

bool valide_GWT_func_message(QByteArray recv, uint8_t low_func);
void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg);

class InverterComm : public QThread
{
    Q_OBJECT
    private:
//        QHidApi *psHandle;
        QTimer *pPoolTimer;
        ProtocolPacket * pcPacket;
        ProtocolGrowatt * gtpcPacket;
        ProtocolGoodWe * gtmodBusPacket;
        ProtocolPacket * pcRecvPacket;
        QSerialPort * porta_serial;
        const uint8_t bMasterAddress = INV_MASTER_ADDRESS;
        bool bDevConnected = false; // mostrando se AP esta conectado com Inversor
        bool bInvRegistered = false;
        uint bDevId;
        uint8_t bGWTfunc;
        uint16_t bGWEfunc;
        bool usedGWTfunc = true;
        bool usedGWeModfunc = true;
        QByteArray mWriteData;
        QByteArray mReadData;
        QDateTime mSystemTime;
        QDateTime mInverterTime;
        int sendData(QByteArray data);
        void handleMessage(uint8_t bCtrlCode, uint8_t bFuncCode, QByteArray mData);
        void handleMessageGrowatt(uint8_t lowReg, uint8_t bFuncCode, QByteArray mData,QByteArray recVexecute);
        void handleMessageGoodWeModbus(uint16_t lowReg, uint8_t bFuncCode, QByteArray mData, QByteArray recVexecute);
        void handleReadyRead(void);
        bool readDevice(void);
    public:
        explicit InverterComm(QObject *parent = nullptr);
        virtual ~InverterComm();
        int initDeviceLayer(Serial_Settings settings);
        bool deInitDeviceLayer();
        void registerConnection(bool registered);
        void unregisterConnection(void);
        void updateInverterConfig(sInverterConfigUI cfg);
        void updatePowFactorConfig(sInverterPowerFactor cfg);
        void updateInverterConfigSerial(Serial_Settings cfg);
        void getInverterAbout(void);
        void getInverterRTC(void);
        void setInverterRTC(void);
        void getInverterInfo(void);
        void getInverterConfig(void);
        void getInverterReconnectTime(void);
        void getPowerFactor(void);
        void getInverterConfigSerial(void);
        void syncInverterRTC(QDateTime currentTime);
        bool bRegister(uint8_t bFuncCode);
        bool bRead(uint8_t bFuncCode);
        bool bReadGT_holding(uint8_t bFuncCode);
        bool bReadGWe_modbus(uint8_t bFuncCode);
        bool bReadGT_input(uint8_t bFuncCode);
        bool bExecute(uint8_t bFuncCode);
        bool bExecuteGwt(uint8_t low_reg);
        bool bExecuteGW_modbus(uint16_t low_reg);
        bool IsInverter();
        bool IsConnected();
        void call_readDevice();
        bool lostConnection();
        void statsFinal();
    protected:
        void run(void);
        int exec(void);
    signals:
        bool connectionLost();
        void closeMain();
        void updateAboutTab(sIDInfoDataGWE info);
        void updateAboutTab(sIDInfoDataGWT info);
        void updatePowerFactor(float powerFac, bool wewratt);
        void updateRecTime(uint16_t recTime, bool wewratt);
        void updateInfoTab(sRunningInfoData info);
        void updateInfoTab(sGWTRunInfoData info);
        void updateConfigTab(sInverterConfigUI config);
        void updateConfigSerialTab(Serial_Settings cfgSerial);
        void updateCurrentRTCTime(sInverterRTCTime time, uint16_t yearOffset);
        void updateStatusBar(QString stat);
        void prepareUI_goodWe_GWT(eDecideInverter check, QByteArray modelo);
        void confirmNewSettings(uint8_t check);
        void confirmNewPowerFactor(bool check);
        void confirmNewRTCTime(uint8_t check);
    private slots:
        void timerPoolDeviceCallback(void);
};
