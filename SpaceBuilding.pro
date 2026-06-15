QT       += core gui serialport
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    common.cpp \
    ethercatthread.cpp \
    main.cpp \
    mainwindow.cpp \
    robotdescription.cpp \
    socketthread.cpp \
    straightlinetrajectorygenerator.cpp \
    vision.cpp

HEADERS += \
    common.h \
    ethercatthread.h \
    mainwindow.h \
    robotdescription.h \
    socketthread.h \
    straightlinetrajectorygenerator.h \
    vision.h

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/local/include/soem

FORMS += \
    mainwindow.ui

unix:!macx: LIBS += -lsoem

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

unix:!macx: LIBS += -L/usr/local/kdl/lib/ -lorocos-kdl
INCLUDEPATH += /usr/local/include/kdl

LIBS += -L/usr/local/lib/ -lruckig
INCLUDEPATH += /usr/local/include

RESOURCES += \
    resources.qrc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
