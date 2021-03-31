#ifndef DIALOGSERIALCONFIG_H
#define DIALOGSERIALCONFIG_H

#include <QDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QLabel>
#include <QtGui>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QSerialPort>
#include <QLineEdit>
#include <QMetaType>
#include "InverterComm.h"
#include "utils.h"

namespace Ui {
class DialogSerialConfig;
}

class DialogSerialConfig : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSerialConfig(QWidget *parent = nullptr);
    ~DialogSerialConfig();

private:
    Ui::DialogSerialConfig *ui;
    Serial_Settings serial_Settings;
public slots:
    void dlg_boxConfigSerialTab();
    void dlg_fillPortsInfo();
    void dlg_updateSettings();
    void dlg_openSerialPort();
    void updateUIconnect();
    void updateUIdisconnect();
    void scanPort_def();
    void dlg_configure_serialPort();


private slots:
    void dlg_showPortInfo(int inx);
    void dlg_apply();
    void dlg_get_SerialConfig();



    void on_DesconectButton_clicked();

signals:
    void Sig_configure_serialPort(Serial_Settings serial_Settings);
    void Sig_disconnect();

};

#endif // DIALOGSERIALCONFIG_H
