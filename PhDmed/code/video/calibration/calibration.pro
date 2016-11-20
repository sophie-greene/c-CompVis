#-------------------------------------------------
#
# Project created by QtCreator 2013-06-15T06:38:10
#
#-------------------------------------------------
QT       += core

TARGET = calibration
TEMPLATE = app

CONFIG += link_pkgconfig
PKGCONFIG += opencv
SOURCES += main.cpp \
    cameracalibrator.cpp
HEADERS += \
    cameracalibrator.h
