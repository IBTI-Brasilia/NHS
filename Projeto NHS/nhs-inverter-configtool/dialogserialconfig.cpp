#include "dialogserialconfig.h"
#include "ui_dialogserialconfig.h"
#include "inverterconfigmain.h"

static const char blankString[] = QT_TRANSLATE_NOOP("SettingsDialog", "N/A");
static int NumSerialPort = 0;

DialogSerialConfig::DialogSerialConfig(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogSerialConfig)
{

    ui->setupUi(this);
    // removendo a flag '?' e deixando o fechar
    Qt::WindowFlags flags = windowFlags();
    Qt::WindowFlags helpFlag = Qt::WindowContextHelpButtonHint;
    flags = flags & (~helpFlag);

    this->setWindowFlags(flags);
    this->setWindowTitle("Configuração da Porta Serial");
    connect(ui->serialPortInfoListBox, SIGNAL(currentIndexChanged(int)), this, SLOT(dlg_showPortInfo(int)));
    connect(ui->applyButton, SIGNAL(clicked()), this, SLOT(dlg_apply()));
    connect(ui->conectSerialButton, SIGNAL(clicked()), this, SLOT(dlg_configure_serialPort()));
    connect(ui->actualParamButton, SIGNAL(clicked()), this, SLOT(dlg_get_SerialConfig()));

    ui->actualParamButton->setDisabled(true);
    ui->DesconectButton->setDisabled(true);
    ui->serialPortInfoListBox->clear();
    dlg_boxConfigSerialTab();
    dlg_fillPortsInfo();
    dlg_updateSettings();
}

DialogSerialConfig::~DialogSerialConfig()
{
    delete ui;
}

void DialogSerialConfig::dlg_boxConfigSerialTab()
{
    ui->baudRateBox->addItem(QStringLiteral("9600"), QSerialPort::Baud9600);
    ui->baudRateBox->addItem(QStringLiteral("19200"), QSerialPort::Baud19200);
    ui->baudRateBox->addItem(QStringLiteral("38400"), QSerialPort::Baud38400);
    ui->baudRateBox->addItem(QStringLiteral("115200"), QSerialPort::Baud115200);

    ui->dataBitsBox->addItem(QStringLiteral("5"), QSerialPort::Data5);
    ui->dataBitsBox->addItem(QStringLiteral("6"), QSerialPort::Data6);
    ui->dataBitsBox->addItem(QStringLiteral("7"), QSerialPort::Data7);
    ui->dataBitsBox->addItem(QStringLiteral("8"), QSerialPort::Data8);
    ui->dataBitsBox->setCurrentIndex(3);

    ui->parityBox->addItem(tr("None"), QSerialPort::NoParity);
    ui->parityBox->addItem(tr("Even"), QSerialPort::EvenParity);
    ui->parityBox->addItem(tr("Odd"), QSerialPort::OddParity);
    ui->parityBox->addItem(tr("Mark"), QSerialPort::MarkParity);
    ui->parityBox->addItem(tr("Space"), QSerialPort::SpaceParity);

    ui->stopBitsBox->addItem(QStringLiteral("1"), QSerialPort::OneStop);
#ifdef Q_OS_WIN
    ui->stopBitsBox->addItem(tr("1.5"), QSerialPort::OneAndHalfStop);
#endif
    ui->stopBitsBox->addItem(QStringLiteral("2"), QSerialPort::TwoStop);

    ui->flowControlBox->addItem(tr("None"), QSerialPort::NoFlowControl);
    ui->flowControlBox->addItem(tr("RTS/CTS"), QSerialPort::HardwareControl);
    ui->flowControlBox->addItem(tr("XON/XOFF"), QSerialPort::SoftwareControl);
}


void DialogSerialConfig::dlg_fillPortsInfo()
{
//    ui->serialPortInfoListBox->clear();
    QString description;
    //  QString manufacturer;
    //  QString serialNumber;
    const auto infos = QSerialPortInfo::availablePorts();
qDebug() << "portas seriais encontradas" << infos.count() << infos.length();
    if(NumSerialPort != infos.length()) {
        qDebug() << "ATUALIZANDO portas seriais encontradas";
        NumSerialPort = infos.length();
        ui->serialPortInfoListBox->clear();
        for (const QSerialPortInfo& info : infos) {
            QStringList list;
            description = info.description();
            //    qDebug() << "vendo informacoes" << info.portName() << " " << info.description() << " " << info.vendorIdentifier() << " " << info.productIdentifier();
            //        manufacturer = info.manufacturer();
            //        serialNumber = info.serialNumber();
            list << info.portName()
                 << (!description.isEmpty() ? description : blankString)
                 << (info.vendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : blankString)
                 << (info.productIdentifier() ? QString::number(info.productIdentifier(), 16) : blankString);

            ui->serialPortInfoListBox->addItem(list.first(), list);
            //        ui->descriptionLabel->setText(tr("Descrição: %1").arg(list.count() > 1 ? list.at(1) : tr(blankString)));
            //        ui->vidLabel->setText(tr("Vendor ID: %1").arg(list.count() > 2 ? list.at(2) : tr(blankString)));
            //        ui->pidLabel->setText(tr("Product ID: %1").arg(list.count() > 3 ? list.at(3) : tr(blankString)));
            //        qDebug() << "lista" << list.first();
        }
    }
    //    ui->serialPortInfoListBox->addItem(tr("Custom"));
}

void DialogSerialConfig::dlg_updateSettings()
{

    serial_Settings.name = ui->serialPortInfoListBox->currentText();
    if (ui->baudRateBox->currentIndex() == 4) {
        serial_Settings.baudRate = ui->baudRateBox->currentText().toInt();
    }
    else {
        serial_Settings.baudRate = static_cast<QSerialPort::BaudRate>(ui->baudRateBox->itemData(ui->baudRateBox->currentIndex()).toInt());
    }
    serial_Settings.stringBaudRate = QString::number(serial_Settings.baudRate);

    serial_Settings.dataBits = static_cast<QSerialPort::DataBits>(ui->dataBitsBox->itemData(ui->dataBitsBox->currentIndex()).toInt());
    serial_Settings.stringDataBits = ui->dataBitsBox->currentText();

    serial_Settings.parity = static_cast<QSerialPort::Parity>(ui->parityBox->itemData(ui->parityBox->currentIndex()).toInt());
    serial_Settings.stringParity = ui->parityBox->currentText();

    serial_Settings.stopBits = static_cast<QSerialPort::StopBits>(
        ui->stopBitsBox->itemData(ui->stopBitsBox->currentIndex()).toInt());
    serial_Settings.stringStopBits = ui->stopBitsBox->currentText();

    serial_Settings.flowControl = static_cast<QSerialPort::FlowControl>(ui->flowControlBox->itemData(ui->flowControlBox->currentIndex()).toInt());
    serial_Settings.stringFlowControl = ui->flowControlBox->currentText();
    //QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_SETTINGS_TITLE, tr("Configuração Atualizada!"));
}

void DialogSerialConfig::dlg_showPortInfo(int idx)
{
    if (idx == -1)
        return;
    const QStringList list = ui->serialPortInfoListBox->itemData(idx).toStringList();
//    ui->descriptionLabel->setText(tr("Descrição: %1").arg(list.count() > 1 ? list.at(1) : tr(blankString)));
//    ui->vidLabel->setText(tr("Vendor ID: %1").arg(list.count() > 2 ? list.at(2) : tr(blankString)));
//    ui->pidLabel->setText(tr("Product ID: %1").arg(list.count() > 3 ? list.at(3) : tr(blankString)));
}

void DialogSerialConfig::dlg_apply()
{
    QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_SETTINGS_TITLE," <strong>Parâmetros atualizados com sucesso!</strong>" );
    dlg_updateSettings();
    // openSerialPort();
    // hide();
}

void DialogSerialConfig::dlg_configure_serialPort()
{
    //  updateSettings();
    dlg_openSerialPort();
//    this->close();
//    DialogSerialConfig::activateWindow();
    // hide();
}

void DialogSerialConfig::dlg_openSerialPort()
{
    emit Sig_configure_serialPort(serial_Settings);
}

void DialogSerialConfig::dlg_get_SerialConfig()
{
    Serial_Settings p = serial_Settings;

    if(!ui->conectSerialButton->isEnabled()){
        QMessageBox::information(this, INV_CFG_QMSGBOX_NEW_SETTINGS_TITLE, tr("<strong>Conectado na porta</strong> %1<strong> com as configurações:</strong><br><br><strong>BaudRate:</strong> %2<br><strong>Data bits:</strong> %3<br><strong>Paridade:</strong> %4<br><strong>Bit de parada:</strong> %5<br><strong>Fluxo:</strong> %6")
                                                                               .arg(p.name)
                                                                               .arg(p.stringBaudRate)
                                                                               .arg(p.stringDataBits)
                                                                               .arg(p.stringParity)
                                                                               .arg(p.stringStopBits)
                                                                               .arg(p.stringFlowControl));
    } else {
    QMessageBox::critical(this, tr("Erro"), "<strong>Não há portas seriais conectadas no momento!<br>Por favor, conecte antes.</strong>");
}
}


void DialogSerialConfig::scanPort_def()
{
//    ui->serialPortInfoListBox->clear();
    dlg_fillPortsInfo();
}

void DialogSerialConfig::updateUIconnect() {

    ui->conectSerialButton->setDisabled(true);
    ui->applyButton->setDisabled(true);
    ui->baudRateBox->setDisabled(true);
    ui->dataBitsBox->setDisabled(true);
    ui->parityBox->setDisabled(true);
    ui->stopBitsBox->setDisabled(true);
    ui->flowControlBox->setDisabled(true);
    ui->serialPortInfoListBox->setDisabled(true);
    ui->DesconectButton->setEnabled(true);
    ui->actualParamButton->setEnabled(true);

}

void DialogSerialConfig::updateUIdisconnect() {

    ui->conectSerialButton->setEnabled(true);
    ui->applyButton->setEnabled(true);
    ui->baudRateBox->setEnabled(true);
    ui->dataBitsBox->setEnabled(true);
    ui->parityBox->setEnabled(true);
    ui->stopBitsBox->setEnabled(true);
    ui->flowControlBox->setEnabled(true);
    ui->serialPortInfoListBox->setEnabled(true);
    ui->DesconectButton->setDisabled(true);
    ui->actualParamButton->setDisabled(true);

}

void DialogSerialConfig::on_DesconectButton_clicked()
{
    emit Sig_disconnect();
    DialogSerialConfig::activateWindow();
}

