# -------------------------------------------------
# Project created by QtCreator 2013-06-12T14:33:38
# -------------------------------------------------
TARGET = gui-cam-cap
TEMPLATE = app
CONFIG += link_pkgconfig
PKGCONFIG += opencv
SOURCES += main.cpp \
    mainwindow.cpp \
    capturethread.cpp \
    calibratedialog.cpp \
    calibratethread.cpp \ # \
    cameracalibrator.cpp

# v4l2uvc.c \
# uvccapture.cpp #\
# framecapture.cpp
HEADERS += mainwindow.h \
    capturethread.h \
    calibratedialog.h \
    calibratethread.h \ # \
    cameracalibrator.h
