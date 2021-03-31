#include "inverterconfigmain.h"
#include "ui_inverterconfigmain.h"

InverterConfigMain::InverterConfigMain(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::InverterConfigMain)
{
    pInvComm = new InverterComm();
    dialog  = new DialogSerialConfig();
    scanTimer = new QTimer();
    connect(pInvComm, SIGNAL(updateAboutTab(sIDInfoDataGWE)), this, SLOT(updateUIAboutTab(sIDInfoDataGWE)));
    connect(pInvComm, SIGNAL(updateAboutTab(sIDInfoDataGWT)), this, SLOT(updateUIAboutTab(sIDInfoDataGWT)));
    connect(pInvComm, SIGNAL(updateInfoTab(sRunningInfoData)), this, SLOT(updateUIInfoTab(sRunningInfoData)));
    connect(pInvComm, SIGNAL(updateInfoTab(sGWTRunInfoData)), this, SLOT(updateUIInfoTab(sGWTRunInfoData)));
    connect(pInvComm, SIGNAL(updateConfigTab(sInverterConfigUI)), this, SLOT(updateUIConfigTab(sInverterConfigUI)));
    connect(pInvComm, SIGNAL(updateCurrentRTCTime(sInverterRTCTime, uint16_t)), this, SLOT(updateUICurrentRTCTime(sInverterRTCTime, uint16_t)));
    connect(pInvComm, SIGNAL(updateStatusBar(QString)), this, SLOT(updateUIStatusBar(QString)));
    connect(pInvComm, SIGNAL(confirmNewSettings(uint8_t)), this, SLOT(confirmNewSettingsUI(uint8_t)));
    connect(pInvComm, SIGNAL(confirmNewRTCTime(uint8_t)), this, SLOT(confirmNewRTCTimeUI(uint8_t)));
    connect(pInvComm, SIGNAL(confirmNewPowerFactor(bool)), this, SLOT(confirmNewPowerFactorUI(bool)));
    connect(pInvComm, SIGNAL(connectionLost()), this, SLOT(on_actionDisconnect_triggered()));
    connect(pInvComm, SIGNAL(updatePowerFactor(float, bool)), this, SLOT(updateUIpowerFactorLabel(float, bool)));
    connect(pInvComm, SIGNAL(updateRecTime(uint16_t, bool)), this, SLOT(updateUIRectTimeLabel(uint16_t, bool)));
    connect(pInvComm, SIGNAL(prepareUI_goodWe_GWT(eDecideInverter, QByteArray)), this, SLOT(prepareUI(eDecideInverter, QByteArray)));
    connect(scanTimer, SIGNAL(timeout()), this, SLOT(on_ScanPorts_triggered()));


    pInvComm->start();
    ui->setupUi(this);
    ui->actionDisconnect->setDisabled(true);

    connect(dialog, SIGNAL(Sig_configure_serialPort(Serial_Settings)), this, SLOT(configure_serialPort(Serial_Settings)));
    connect(dialog, SIGNAL(Sig_disconnect()), this, SLOT(on_actionDisconnect_triggered()));

    pStatLabel = new QLabel(INV_COMM_INVERTER_DISCONNECTED_TEXT);
    ui->statusBar->addPermanentWidget(pStatLabel);
    ui->labelOverVoltageValue->setText(QString::number(static_cast<double>(INV_SETTING_220V_MAX_GRID_VOLTAGE_VALUE * 0.1)) + " V");
    ui->labelUnderVoltageValue->setText(QString::number(static_cast<double>(INV_SETTING_220V_MIN_GRID_VOLTAGE_VALUE * 0.1)) + " V");
    ui->labelOverFrequencyValue->setText(QString::number(static_cast<double>(INV_SETTING_MAX_GRID_FREQUENCY_VALUE * 0.01)) + " Hz");
    ui->labelUnderFrequencyValue->setText(QString::number(static_cast<double>(INV_SETTING_MIN_GRID_FREQUENCY_VALUE * 0.01)) + " Hz");
    ui->pushButtonConfigure->setDisabled(true);
    ui->pushButtonSync->setDisabled(true);
    ui->pushButtonConfPowerFactor->setDisabled(true);
    ui->comboBoxOutputVoltage->setDisabled(true);
    ui->comboBoxPowerFactor->setDisabled(true);
    ui->doubleSpinBoxPowerFactor->setDisabled(true);
    ui->lineEditStartDelay->setDisabled(true);
    ui->lineEditReconnectTime->setDisabled(true);

//    ui->groupBoxpowerFactor->hide();
//    ui->groupBoxRTC->hide();

//    ui->widget2->hide();
//    ui->widget3->hide();

    scanTimer->start(1500);
//    static bool syncButtonPushed = false;
}

InverterConfigMain::~InverterConfigMain()
{
    delete ui;
}

void InverterConfigMain::cleanUIFields()
{
    ui->fwVersionLineEdit->setText(QString());
    ui->modelLineEdit->setText(QString());
    ui->serialNumberLineEdit->setText(QString());
    ui->labelOutputVoltageValue->setText(QString());
    ui->labelStartDelayValue->setText(QString());
    ui->labelReconnectTime->setText(QString());
    ui->labelDateTime->setText(QString());
    ui->labelPV1VoltageValue->setText(QString("0.0"));
    ui->labelPV2VoltageValue->setText(QString("0.0"));
    ui->labelPV1CurrentValue->setText(QString("0.0"));
    ui->labelPV2CurrentValue->setText(QString("0.0"));
    ui->labelCurrentL1Value->setText(QString("0.0"));
    ui->labelCurrentL2Value->setText(QString("0.0"));
    ui->labelCurrentL3Value->setText(QString("0.0"));
    ui->labelVoltageL1Value->setText(QString("0.0"));
    ui->labelVoltageL2Value->setText(QString("0.0"));
    ui->labelVoltageL3Value->setText(QString("0.0"));
    ui->labelFrequencyL1Value->setText(QString("0.0"));
    ui->labelFrequencyL2Value->setText(QString("0.0"));
    ui->labelFrequencyL3Value->setText(QString("0.0"));
    ui->labelTemperatureValue->setText(QString("0.0"));
    ui->labelWorkModeValue->setText(QString(""));
    ui->labelCountryCodeValue->setText(QString(""));
    ui->labelFeedEnergyValue->setText(QString("0.0"));
    ui->labelFeedingHoursValue->setText(QString("0"));
    ui->labelErrorCodeValue->setText(QString("00000000"));
}

void InverterConfigMain::closeEvent (QCloseEvent *event){

    pInvComm->statsFinal();
    event->accept();

}

void InverterConfigMain::updateUIAboutTab(sIDInfoDataGWE info)
{
    ui->fwVersionLineEdit->setText(QString(reinterpret_cast<const char*>(info.abFWVersion)));
    ui->modelLineEdit->setText(QString(reinterpret_cast<const char*>(info.abModelName)));
    ui->serialNumberLineEdit->setText(QString(reinterpret_cast<const char*>(info.abSerialNumber)));
}

void InverterConfigMain::updateUIAboutTab(sIDInfoDataGWT info)
{
    ui->fwVersionLineEdit->setText(QString(reinterpret_cast<const char*>(info.abFWVersion)));
    ui->modelLineEdit->setText("GT"+QString("%1%2%3%4").arg(info.abModelName[0],2,16,QLatin1Char('0')).toUpper().arg(info.abModelName[1],2,16,QLatin1Char('0')).toUpper().arg(info.abModelName[2],2,16,QLatin1Char('0')).toUpper().arg(info.abModelName[3],2,16,QLatin1Char('0')).toUpper()); //Fazer GT+hexa
    ui->serialNumberLineEdit->setText(QString(reinterpret_cast<const char*>(info.abSerialNumber)));

}


void InverterConfigMain::updateUIpowerFactorLabel(float info, bool wewratt)
{
    qDebug() << "--- power factor ->" << info;
    QString aux;
    float fator;
    if(wewratt) {
        if(info == 2.0f) {
            ui->labelPowerFactorCurveValue->setText(QString("Desabilitada"));
            ui->labelPowerFactorValue->setText(QString("Unitário"));
        } else if (info <= 1.0f) {
            fator = 1.0f-info;
            ui->labelPowerFactorCurveValue->setText(QString("Atrasada"));
            ui->labelPowerFactorValue->setText(aux.sprintf("%03.4f", static_cast<double>(fator)));
        } else if (info > 1.0f && info <= 2.0f) {
            ui->labelPowerFactorCurveValue->setText(QString("Adiantada"));
            ui->labelPowerFactorValue->setText(aux.sprintf("%03.4f", static_cast<double>(info)));
        }   else {
            ui->labelPowerFactorCurveValue->setText(QString("Fora do range"));
            ui->labelPowerFactorValue->setText(aux.sprintf("%03.4f", static_cast<double>(info)));
        }


    } else {
    if (info == 1.0f || info == 0.0f || info == 2.0f) {
        ui->labelPowerFactorCurveValue->setText(QString("Desabilitada"));
        ui->labelPowerFactorValue->setText(QString("Unitário"));
    } else if (info < 1.0f) {
        fator = 1.0f-info;
        ui->labelPowerFactorCurveValue->setText(QString("Subexcitado"));
        ui->labelPowerFactorValue->setText(aux.sprintf("%03.4f Capacitiva", static_cast<double>(fator)));

        // ou seja eh reativa -- capacitiva.
    } else if (info > 1.0f && info <= 2.0f) {
        fator = info - 1.0f;
        ui->labelPowerFactorCurveValue->setText(QString("Sobre-excitado"));
        ui->labelPowerFactorValue->setText(aux.sprintf("%03.4f Indutivo", static_cast<double>(fator)));
        // ou seja eh reativa -- capacitiva.
}    else {
        ui->labelPowerFactorCurveValue->setText(QString("Fora do range"));
        ui->labelPowerFactorValue->setText(aux.sprintf("%03.4f", static_cast<double>(info)));
    }
}
}

void InverterConfigMain::updateUIRectTimeLabel(uint16_t info, bool wewratt)
{
    qDebug() << "--- Reconnection Timer";
    QString aux;
    if(!wewratt) {
       ui->labelReconnectTime->setText(QString::number(info));
    }
}

void InverterConfigMain::updateUIInfoTab(sRunningInfoData info)
{
    QString aux;
    ui->labelPV1CurrentValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV1Current)));
    ui->labelPV1VoltageValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV1Voltage)));
    ui->labelPV2CurrentValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV2Current)));
    ui->labelPV2VoltageValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV2Voltage)));

    ui->labelVoltageL1Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL1Voltage)));
    ui->labelVoltageL2Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL2Voltage)));
    ui->labelVoltageL3Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL3Voltage)));

    ui->labelCurrentL1Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL1Current)));
    ui->labelCurrentL2Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL2Current)));
    ui->labelCurrentL3Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL3Current)));

    ui->labelFrequencyL1Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL1Frequency)));
    ui->labelFrequencyL2Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL2Frequency)));
    ui->labelFrequencyL3Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL3Frequency)));

    ui->labelTemperatureValue->setText(aux.sprintf("%02.1f", static_cast<double>(info.fInternalTemp)));

    ui->labelFeedingHoursValue->setText(aux.sprintf("%02.1f", static_cast<double>(info.dTotalFeedHours)));
    ui->labelFeedEnergyValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fTotalEnergy2Grid)));

    aux = (static_cast<eInverterWorkMode>(info.eInvWorkMode == INV_WORK_MODE_NORMAL)) ? QString("Normal") : (static_cast<eInverterWorkMode>(info.eInvWorkMode == INV_WORK_MODE_FAULT)) ? QString("Erro") : QString("Aguardando");
    ui->labelWorkModeValue->setText(aux);

    ui->labelCountryCodeValue->setText((info.bSafetyCountryCode == 0x10) ? QString("Brasil") : QString("Erro"));

    ui->labelErrorCodeValue->setText(aux.sprintf("%08X", info.dErrorMessage));

    if ((info.bPowerFactor > 10) && (info.bPowerFactor < 97)) {
        ui->labelPowerFactorCurveValue->setText(QString("Desabilitada"));
        ui->labelPowerFactorValue->setText(QString::number(info.bPowerFactor));
    }
    else {
        ui->labelPowerFactorCurveValue->setText(QString("Desabilitada"));
        ui->labelPowerFactorValue->setText(QString("Unitário"));
    }
}

void InverterConfigMain::updateUIInfoTab(sGWTRunInfoData info)
{
    QString aux;
    ui->labelPV1CurrentValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV1Current)));
    ui->labelPV1VoltageValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV1Voltage)));
    ui->labelPV2CurrentValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV2Current)));
    ui->labelPV2VoltageValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPV2Voltage)));

    ui->labelVoltageL1Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL1Voltage)));
    ui->labelVoltageL2Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL2Voltage)));
    ui->labelVoltageL3Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL3Voltage)));

    ui->labelCurrentL1Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL1Current)));
    ui->labelCurrentL2Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL2Current)));
    ui->labelCurrentL3Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL3Current)));

    ui->labelFrequencyL1Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL1Frequency)));
    ui->labelFrequencyL2Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL2Frequency)));
    ui->labelFrequencyL3Value->setText(aux.sprintf("%03.1f", static_cast<double>(info.fPhaseL3Frequency)));

    ui->labelTemperatureValue->setText(aux.sprintf("%02.1f", static_cast<double>(info.fInternalTemp)));

    ui->labelFeedingHoursValue->setText(aux.sprintf("%02.1f", static_cast<double>(info.dTotalFeedHours)));
    ui->labelFeedEnergyValue->setText(aux.sprintf("%03.1f", static_cast<double>(info.fTotalEnergy2Grid)));

    aux = (static_cast<eInvGWTWorkMode_tDef>(info.eInvGWTWorkMode == GWT_WORK_MODE_NORMAL)) ? QString("Normal") : (static_cast<eInverterWorkMode>(info.eInvGWTWorkMode == GWT_WORK_MODE_FAULT)) ? QString("Erro") : QString("Aguardando");
    ui->labelWorkModeValue->setText(aux);

    // nao informacao parecida
    ui->labelCountryCodeValue->setText(QString("Brasil"));

    ui->labelErrorCodeValue->setText(aux.sprintf("%04X", info.dErrorMessage));

//    if ((info.bPowerFactor > 10) && (info.bPowerFactor < 97)) {
//        ui->labelPowerFactorCurveValue->setText(QString("Habilitada"));
//        ui->labelPowerFactorValue->setText(QString::number(info.bPowerFactor));
//    }
//    else {
//        ui->labelPowerFactorCurveValue->setText(QString("Desabilitada"));
//        ui->labelPowerFactorValue->setText(QString("Unitário"));
//    }
}

void InverterConfigMain::updateUIConfigTab(sInverterConfigUI config)
{
    QString mOutputVotlage;

    switch (config.eOutVoltage) {
    case OUTPUT_VOLTAGE_220_127V: {
        mOutputVotlage = "220/127V";
        break;
    }
    case OUTPUT_VOLTAGE_230_115V: {
        mOutputVotlage = "230/115V";
        break;
    }
    case OUTPUT_VOLTAGE_240_120V: {
        mOutputVotlage = "240/120V";
        break;
    }
    case OUTPUT_VOLTAGE_254_127V: {
        mOutputVotlage = "254/127V";
        break;
    }
    default:
        mOutputVotlage = "Erro";
        break;
    }
    config.wMaxGridFrequency = INV_SETTING_MAX_GRID_FREQUENCY_VALUE;
    config.wMinGridFrequency = INV_SETTING_MIN_GRID_FREQUENCY_VALUE;
    ui->labelOutputVoltageValue->setText(mOutputVotlage);
    ui->labelStartDelayValue->setText(QString::number(config.wStartDelay));
    updateDetailedConfiguration(config.eOutVoltage);
    ui->labelOverFrequencyValue->setText(QString::number(static_cast<double>((config.wMaxGridFrequency * 0.01))) + " Hz");
    ui->labelUnderFrequencyValue->setText(QString::number(static_cast<double>((config.wMinGridFrequency * 0.01))) + " Hz");

}

void InverterConfigMain::updateUICurrentRTCTime(sInverterRTCTime time, uint16_t yearOffset)
{
    QString mDateTimeText;
    uint16_t wYear = uint16_t(time.bYear) + yearOffset;
    QDateTime mTime(QDate(wYear, time.bMonth, time.bDay), QTime(time.bHour, time.bMinute));
    if(wYear == 0 || time.bMonth == 0 || time.bMonth > 12 || time.bDay == 0 || time.bDay > 31){

        ui->labelDateTime->setText(QString("Erro").toUpper());
        if(pInvComm->IsConnected() && syncButtonPushed){
            QMessageBox::critical(this, INV_CFG_QMSGBOX_NEW_RTC_TIME_TITLE, INV_CFG_QMSGBOX_NEW_RTC_TIME_ERROR_TEXT);
        }

    }else{
        mDateTimeText.append(mTime.date().toString("dd/MM/yyyy"));
        mDateTimeText.append("  ");
        mDateTimeText.append(mTime.time().toString("HH:mm"));

        ui->labelDateTime->setText(mDateTimeText);
    }
    syncButtonPushed = false;
}

void InverterConfigMain::updateUIStatusBar(QString stat)
{
    pStatLabel->setText(stat);
}

void InverterConfigMain::confirmNewPowerFactorUI(bool check)
{
    if (check) {
        QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_POW_FAC_TITLE, INV_CFG_QMSGBOX_NEW_POW_FAC_TEXT);
    }
    else {
        QMessageBox::critical(this, INV_CFG_QMSGBOX_NEW_POW_FAC_TITLE, INV_CFG_QMSGBOX_NEW_POW_FAC_ERROR_TEXT);
    }
}

void InverterConfigMain::prepareUI(eDecideInverter check, QByteArray modelo)
{
    if (check == INV_GOODWE ) {
        // inversor GoodWe --
        ui->groupBoxpowerFactor->hide();
        ui->lineEditReconnectTime->hide();
        ui->labelReconnectTime->hide();
        ui->labelReconnectTime_2->hide();
        ui->labelRecconetTime->hide();
        ui->labelRecconetTimeRange->hide();
        ui->groupBoxRTC->show();


    } else if(check == INV_GOODWE_MOD) {
        // inversor GoodWe MODBUS
        //-- Nao tem fw version e em tese pode ter power factor
        ui->lineEditReconnectTime->hide();
        ui->labelReconnectTime->hide();
        ui->labelReconnectTime_2->hide();
        ui->labelRecconetTime->hide();
        ui->labelRecconetTimeRange->hide();
        ui->fwVersionLabel->hide();
        ui->fwVersionLineEdit->hide();
        ui->groupBoxRTC->show();
        ui->groupBoxpowerFactor->show();
        ui->label_26->hide();
        ui->labelCountryCodeValue->hide();

    }
    else if (check == INV_GROWATT) {
       /* inversor Growatt
        * removendo pais/country ja que nao eh configurado ...
        * Modelos growatt da NHS
        * 1500 - 1000E111 (NÃO TEM POWER FACTOR NEM RTC POSSIVELMENTE)
        * 3000 - 1000E131 (NÃO TEM POWER FACTOR NEM RTC!)
        * 5000 - 1010E165 (TEM POWER FACTOR E TEM RTC! )
        */
        ui->label_26->hide();
        ui->labelCountryCodeValue->hide();
        // inversor 5000
        if((static_cast<uint8_t>(modelo.at(0)) == 0x10) && (static_cast<uint8_t>(modelo.at(1)) == 0x10) && (static_cast<uint8_t>(modelo.at(2)) == 0xE1) && (static_cast<uint8_t>(modelo.at(3)) == 0x65)){
            ui->groupBoxRTC->show();
            ui->groupBoxpowerFactor->show();
            qDebug()<<"model 5000: " << modelo.toHex();

        }
        // inversor 3000
        else if ((static_cast<uint8_t>(modelo.at(0)) == 0x10) && (static_cast<uint8_t>(modelo.at(1)) == 0x00) && (static_cast<uint8_t>(modelo.at(2)) == 0xE1) && (static_cast<uint8_t>(modelo.at(3)) == 0x31)) {
            ui->groupBoxRTC->hide();
            ui->groupBoxpowerFactor->hide();
            qDebug()<<"model 3000: " << modelo.toHex();
        }
        // inversor 1500
        else if ((static_cast<uint8_t>(modelo.at(0)) == 0x10) && (static_cast<uint8_t>(modelo.at(1)) == 0x00) && (static_cast<uint8_t>(modelo.at(2)) == 0xE1) && (static_cast<uint8_t>(modelo.at(3)) == 0x11)) {
            ui->groupBoxRTC->hide();
            ui->groupBoxpowerFactor->hide();
            qDebug()<<"model 1500: " << modelo.toHex();
        }
        else {
            ui->groupBoxRTC->show();
            ui->groupBoxpowerFactor->show();
            qDebug()<<"model permissivo -- deixar tudo: " << modelo.toHex();
}

     } else if (check == INV_DEC_ERRO) {
        // ERRO AO IDENTIFICAR
        pInvComm->deInitDeviceLayer();

    }

}

void InverterConfigMain::confirmNewSettingsUI(uint8_t check)
{
    if (check == INV_EXEC_ACKNOWLEDGEMENT_ACK) {
        QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_SETTINGS_TITLE, INV_CFG_QMSGBOX_NEW_SETTINGS_TEXT);
    }
    else if ((check == INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL) || (check == INV_EXEC_ACKNOWLEDGEMENT_NACK)) {
        QMessageBox::critical(this, INV_CFG_QMSGBOX_NEW_SETTINGS_TITLE, INV_CFG_QMSGBOX_NEW_SETTINGS_ERROR_TEXT);
    }
}

void InverterConfigMain::confirmNewRTCTimeUI(uint8_t check)
{
    if (check == INV_EXEC_ACKNOWLEDGEMENT_ACK) {
        QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_RTC_TIME_TITLE, INV_CFG_QMSGBOX_NEW_RTC_TIME_TEXT);
    }
    else if ((check == INV_EXEC_ACKNOWLEDGEMENT_CHECKSUM_FAIL) || (check == INV_EXEC_ACKNOWLEDGEMENT_NACK)) {
        QMessageBox::critical(this, INV_CFG_QMSGBOX_NEW_RTC_TIME_TITLE, INV_CFG_QMSGBOX_NEW_RTC_TIME_ERROR_TEXT);
    }
}

void InverterConfigMain::on_actionConnect_triggered()
{
//    Serial_Settings p = serial_Settings;
//    if (pInvComm->initDeviceLayer(serial_Settings)) {
//        QMessageBox::information(this, "Conectado com sucesso!", tr("\t\tConectado em %1\n\t<strong>baudRate: %2</strong>\n\t<strong>Data bits:</strong> %3\n\t<strong>Paridade:</strong> %4\n\t<strong>Bit de parada:</strong> %5\n\t<strong>Fluxo: %6</strong>")
//                                                                               .arg(p.name)
//                                                                               .arg(p.stringBaudRate)
//                                                                               .arg(p.stringDataBits)
//                                                                               .arg(p.stringParity)
//                                                                               .arg(p.stringStopBits)
//                                                                               .arg(p.stringFlowControl));
//        ui->conectSerialButton->setDisabled(true);
//        ui->actionConnect->setDisabled(true);
//        ui->ScanPorts->setDisabled(true);
//        ui->applyButton->setDisabled(true);
//        ui->actionDisconnect->setEnabled(true);
//        ui->baudRateBox->setDisabled(true);
//        ui->dataBitsBox->setDisabled(true);
//        ui->parityBox->setDisabled(true);
//        ui->stopBitsBox->setDisabled(true);
//        ui->flowControlBox->setDisabled(true);
//        ui->serialPortInfoListBox->setDisabled(true);

//    }
//    else {
//        QMessageBox::critical(this, tr("Erro ao abrir"), "<strong>Inversor não encontrado ou não encontrado</strong>");
//    }
    dialog->dlg_configure_serialPort();
}

void InverterConfigMain::on_actionDisconnect_triggered()
{
    if (pInvComm->deInitDeviceLayer()) {
        cleanUIFields();
        ui->pushButtonConfigure->setDisabled(true);
        ui->pushButtonSync->setDisabled(true);
        ui->pushButtonConfPowerFactor->setDisabled(true);
        ui->comboBoxOutputVoltage->setDisabled(true);
        ui->comboBoxPowerFactor->setDisabled(true);
        ui->doubleSpinBoxPowerFactor->setDisabled(true);
        ui->actionConnect->setEnabled(true);
        ui->ScanPorts->setEnabled(true);
        ui->actionDisconnect->setDisabled(true);
        ui->actionSerialConfig->setEnabled(true);
        ui->lineEditStartDelay->setDisabled(true);
        ui->lineEditReconnectTime->setDisabled(true);
        ui->widget2->hide();
        ui->widget3->hide();
        dialog->updateUIdisconnect();
        QMessageBox::critical(this, tr("Inversor Desconectado. "), "<strong>Conexão com o inversor encerrada.</strong>");
        scanTimer->start(1500);
    }
}

void InverterConfigMain::on_pushButtonConfigure_clicked()
{
    sInverterConfigUI eNewConfig;
    uint16_t wDelay = static_cast<uint16_t>(QString(ui->lineEditStartDelay->text()).toInt());
    uint16_t wReconnect =static_cast<uint16_t>(QString(ui->lineEditReconnectTime->text()).toInt());



    if ((wDelay >= INV_CONFIG_OUTPUT_VOLTAGE_RANGE_MIN) && (wDelay <= INV_CONFIG_OUTPUT_VOLTAGE_RANGE_MAX)) {
        if((!ui->lineEditReconnectTime->isVisible()) || ((wReconnect >= INV_CONFIG_OUTPUT_VOLTAGE_RANGE_MIN) && (wReconnect <= INV_CONFIG_OUTPUT_VOLTAGE_RANGE_MAX))) {
            eNewConfig.wStartDelay = wDelay;
            eNewConfig.eOutVoltage = static_cast<eOutputVoltage>(ui->comboBoxOutputVoltage->currentIndex());
            eNewConfig.wReconnectTime = wReconnect;
            // ou serah que eh zero
            pInvComm->updateInverterConfig(eNewConfig);
        } else {
            QMessageBox::information(this, INV_CFG_QMSGBOX_INVALID_RANGE_TITLE, INV_CFG_QMSGBOX_INVALID_RANGE_TEXT);

}
    }
    else {
        QMessageBox::information(this, INV_CFG_QMSGBOX_INVALID_RANGE_TITLE, INV_CFG_QMSGBOX_INVALID_RANGE_TEXT);
    }
}

void InverterConfigMain::on_tabWidget_currentChanged(){}

void InverterConfigMain::on_pushButtonSync_clicked()
{
    syncButtonPushed = true;
    pInvComm->syncInverterRTC(QDateTime(QDate::currentDate(), QTime::currentTime()));
}

void InverterConfigMain::on_comboBoxOutputVoltage_currentIndexChanged(int index)
{
    (void)index;
}

void InverterConfigMain::updateDetailedConfiguration(eOutputVoltage voltage)
{
    switch (voltage) {
    case OUTPUT_VOLTAGE_220_127V: {
        ui->labelOverVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_220V_MAX_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        ui->labelUnderVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_220V_MIN_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        break;
    }
    case OUTPUT_VOLTAGE_230_115V: {
        ui->labelOverVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_230V_MAX_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        ui->labelUnderVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_230V_MIN_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        break;
    }
    case OUTPUT_VOLTAGE_240_120V: {
        ui->labelOverVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_240V_MAX_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        ui->labelUnderVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_240V_MIN_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        break;
    }
    case OUTPUT_VOLTAGE_254_127V: {
        ui->labelOverVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_254V_MAX_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        ui->labelUnderVoltageValue->setText(QString::number(static_cast<double>((INV_SETTING_254V_MIN_GRID_VOLTAGE_VALUE * 0.1))) + " V");
        break;
    }
    default:
        break;
    }
}

void InverterConfigMain::configure_serialPort(Serial_Settings info)
{
    //  updateSettings();
    openSerialPort(info);
    // hide();
}

void InverterConfigMain::openSerialPort(Serial_Settings info)
{
    Serial_Settings p = info;
    int init_ret = pInvComm->initDeviceLayer(p);
    if (init_ret == CONNECTED) {
        QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_OPEN_SERIAL_PORT, tr("\t\t<strong>Conectado em:</strong> %1<br><strong>baudRate:</strong> %2<br> <strong>Data bits:</strong> %3<br><strong>Paridade:</strong> %4<br> <strong>Bit de parada:</strong> %5<br><strong>Fluxo:</strong> %6")
                                                                               .arg(p.name)
                                                                               .arg(p.stringBaudRate)
                                                                               .arg(p.stringDataBits)
                                                                               .arg(p.stringParity)
                                                                               .arg(p.stringStopBits)
                                                                               .arg(p.stringFlowControl));

        ui->pushButtonSync->setEnabled(true);
        ui->pushButtonConfigure->setEnabled(true);
        ui->comboBoxOutputVoltage->setEnabled(true);
        ui->comboBoxPowerFactor->setEnabled(true);
        ui->doubleSpinBoxPowerFactor->setEnabled(true);

        ui->actionConnect->setDisabled(true);
        ui->ScanPorts->setDisabled(true);
        ui->actionDisconnect->setEnabled(true);
        ui->actionSerialConfig->setEnabled(true);
        ui->pushButtonConfPowerFactor->setEnabled(true);
        ui->lineEditStartDelay->setEnabled(true);
        ui->lineEditReconnectTime->setEnabled(true);
        ui->widget2->show();
        ui->widget3->show();
        dialog->updateUIconnect();
        dialog->close();
        scanTimer->stop();
    }
    else if(init_ret == PORT_ERROR){
        QMessageBox::critical(this, tr("Erro ao abrir "), "<strong>Porta não encontrada.</strong>");
        dialog->activateWindow();
    }else if(init_ret == INV_ERROR){
        QMessageBox::critical(this, tr("Erro ao abrir "), "<strong>Inversor não encontrado.</strong>");
        dialog->activateWindow();
    }
}

void InverterConfigMain::on_ScanPorts_triggered(){
//  dialog->show();
    qDebug() << "timeout !!!! plim plim plim";
    dialog->scanPort_def();
    //scanTimer->start(1500);

}

void InverterConfigMain::on_actionSerialConfig_triggered(){

dialog->exec();

}

void InverterConfigMain::on_pushButtonConfPowerFactor_clicked()
{

    sInverterPowerFactor eNewPFConfig;
    float fpowFactor = static_cast<float>(ui->doubleSpinBoxPowerFactor->value());
    if ((fpowFactor <= 1.0f) && (fpowFactor >= 0.0f)) {
        eNewPFConfig.fpowerFactor = fpowFactor;
        eNewPFConfig.eMode = static_cast<ePowerFactorMode>(ui->comboBoxPowerFactor->currentIndex());
        pInvComm->updatePowFactorConfig(eNewPFConfig);
    }
    else {
        QMessageBox::information(this, INV_CFG_QMSGBOX_INVALID_RANGE_TITLE, INV_CFG_QMSPOWFACTOR_INVALID_RANGE_TEXT);
    }

}
