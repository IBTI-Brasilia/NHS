#-------------------------------------------------
#
# Project created by QtCreator 2018-02-06T10:43:45
#
#-------------------------------------------------
INCLUDEPATH += qhidapi/
QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = NHSInverterConfig
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        ProtocolGoodWe.cpp \
        ProtocolGrowatt.cpp \
        dialogserialconfig.cpp \
        main.cpp \
        inverterconfigmain.cpp \
        InverterComm.cpp \
        ProtocolPacket.cpp

HEADERS += \
        ProtocolGoodWe.h \
        ProtocolGrowatt.h \
    dialogserialconfig.h \
        inverterconfigmain.h \
#        qhidapi/hidapi.h \
        utils.h \
        InverterComm.h \
        ProtocolPacket.h

FORMS += \
        dialogserialconfig.ui \
        inverterconfigmain.ui

#SOURCES += \
#    qhidapi/qhidapi.cpp \
#    qhidapi/qhiddeviceinfomodel.cpp \
#    qhidapi/qhidapi_p.cpp \
#    qhidapi/hexformatdelegate.cpp \
#    qhidapi/qhiddeviceinfoview.cpp

#HEADERS += \
#    qhidapi/qhidapi_global.h \
#    qhidapi/qhidapi.h \
#    qhidapi/qhiddeviceinfomodel.h \
#    qhidapi/qhiddeviceinfo.h \
#    qhidapi/qhidapi_p.h \
#    qhidapi/hexformatdelegate.h \
#    qhidapi/qhiddeviceinfoview.h

#macx:  SOURCES += qhidapi/mac/hid.c
#unix: !macx:  SOURCES += qhidapi/linux/hid-libusb.c
#win32: SOURCES += qhidapi/windows/hid.c

macx: LIBS += -framework CoreFoundation -framework IOkit
unix: !macx: LIBS += -lusb-1.0
win32: LIBS += -lSetupAPI

RESOURCES += \
    inverterconfig.qrc

RC_ICONS = img/repairing_service.ico
