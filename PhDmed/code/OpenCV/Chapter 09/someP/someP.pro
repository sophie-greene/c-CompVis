# -------------------------------------------------
# Project created by QtCreator 2013-06-14T07:24:01
# -------------------------------------------------
QT -= gui
TARGET = someP
CONFIG += console
CONFIG -= app_bundle
CONFIG += link_pkgconfig
PKGCONFIG += opencv
TEMPLATE = app
SOURCES += CameraCalibrator.cpp \
    calibrate.cpp
   # estimateH.cpp \#robustmatching.cpp \
    #estimateF.cpp \

HEADERS += matcher.h \
    CameraCalibrator.h
