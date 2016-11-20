#-------------------------------------------------
#
# Project created by QtCreator 2013-06-18T07:33:30
#
#-------------------------------------------------
QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets
include(3rdParty/src/qextserialport.pri)
TARGET = simpleimagecopier
TEMPLATE = app
CONFIG += link_pkgconfig

PKGCONFIG += opencv
INCLUDEPATH += /usr/not-backed-up/OpenCV-2.4-/include/opencv
INCLUDEPATH += /usr/not-backed-up/OpenCV-2.4-/include/opencv2
INCLUDEPATH += /usr/not-backed-up/OpenCV-2.4-/include

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
