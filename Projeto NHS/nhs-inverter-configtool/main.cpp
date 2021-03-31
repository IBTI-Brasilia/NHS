#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <QApplication>
#include <QDebug>
#include "inverterconfigmain.h"
#include <QMetaType>
using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<sRunningInfoData>("sRunningInfoData");
    qRegisterMetaType<sGWTRunInfoData>("sGWTRunInfoData");
    qRegisterMetaType<sInverterConfigUI>("sInverterConfigUI");
    qRegisterMetaType<sInverterRTCTime>("sInverterRTCTime");
    qRegisterMetaType<uint8_t>("uint8_t");
    qRegisterMetaType<uint16_t>("uint16_t");
    InverterConfigMain w;
    w.show();

    return a.exec();
}
