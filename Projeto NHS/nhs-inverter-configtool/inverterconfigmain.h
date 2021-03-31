
#ifndef INVERTERCONFIGMAIN_H
#define INVERTERCONFIGMAIN_H

#include <QMainWindow>
#include <QMessageBox>
#include <QLabel>
#include <QCloseEvent>
#include <QtGui>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QSerialPort>
#include <QLineEdit>
#include <QMetaType>
#include "InverterComm.h"
#include "utils.h"
#include "dialogserialconfig.h"

namespace Ui {
class InverterConfigMain;
}

class InverterConfigMain : public QMainWindow
{
    Q_OBJECT
    public:
        explicit InverterConfigMain(QWidget *parent = nullptr);
        ~InverterConfigMain();
        QTimer *scanTimer;
    private:
        InverterComm* pInvComm;
        QLabel* pStatLabel;
        Ui::InverterConfigMain *ui;
        Serial_Settings serial_Settings;
        DialogSerialConfig * dialog;
        bool syncButtonPushed = false;

        void cleanUIFields(void);
    public slots:
        void updateUIAboutTab(sIDInfoDataGWE info);
        void updateUIAboutTab(sIDInfoDataGWT info);
        void updateUIInfoTab(sRunningInfoData info);
        void updateUIInfoTab(sGWTRunInfoData info);
        void updateUIpowerFactorLabel(float info, bool wewratt);
        void updateUIRectTimeLabel(uint16_t info, bool wewratt);
        void updateUIConfigTab(sInverterConfigUI config);
        void updateUICurrentRTCTime(sInverterRTCTime time, uint16_t yearOffset);
        void updateUIStatusBar(QString stat);
        void confirmNewSettingsUI(uint8_t check);
        void prepareUI(eDecideInverter check, QByteArray modelo);
        void confirmNewRTCTimeUI(uint8_t check);
        void confirmNewPowerFactorUI(bool check);
        void openSerialPort(Serial_Settings info);
        void closeEvent(QCloseEvent *event);
    private slots:
        void on_actionConnect_triggered();
        void on_actionDisconnect_triggered();
        void on_pushButtonConfigure_clicked();
        void on_tabWidget_currentChanged();
        void on_pushButtonSync_clicked();
        void on_comboBoxOutputVoltage_currentIndexChanged(int index);
        void updateDetailedConfiguration(eOutputVoltage voltage);
        void configure_serialPort(Serial_Settings info);
        void on_ScanPorts_triggered();
        void on_actionSerialConfig_triggered();
        void on_pushButtonConfPowerFactor_clicked();
};

#endif // INVERTERCONFIGMAIN_H
