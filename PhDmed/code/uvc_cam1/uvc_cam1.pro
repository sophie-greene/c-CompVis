# -------------------------------------------------
# Project created by QtCreator 2012-12-10T13:54:09
# -------------------------------------------------
QT -= gui
TARGET = uvc_cam1
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
SOURCES += main.cpp \
    timer.c \
    support.c \
    option.c \
    monstring.c \
    memory.c \
    interface.c \
    input-uvc.c \
    image.c \
    device.c \
    configfile.c \
    callbacks.c \
    main.c
OTHER_FILES += uvccapture.conf
HEADERS += timer.h \
    support.h \
    option.h \
    monstring.h \
    memory.h \
    interface.h \
    input-uvc.h \
    image.h \
    geometric.h \
    device.h \
    debug.h \
    configfile.h \
    callbacks.h
