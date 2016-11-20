# -------------------------------------------------
# Project created by QtCreator 2012-12-07T15:05:57
# -------------------------------------------------
TARGET = uvcCam
TEMPLATE = app
SOURCES += main.cpp \
    mainwindow.cpp \
    v4l2uvc.c
HEADERS += mainwindow.h \
    v4l2uvc.h
FORMS += mainwindow.ui

CONFIG += link_pkgconfig
PKGCONFIG += opencv
