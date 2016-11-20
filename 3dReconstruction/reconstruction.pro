QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets
include(3rdParty/src/qextserialport.pri)
TARGET = reconstruction
TEMPLATE = app
CONFIG += link_pkgconfig
PKGCONFIG += opencv
INCLUDEPATH += /usr/include/opencv
INCLUDEPATH += /usr/include/opencv2
INCLUDEPATH += /usr/include
SOURCES += src/main.cpp \
    src/mainwindow.cpp \
    src/capturethread.cpp \
    src/calibratethread.cpp \
    src/cameracalibrator.cpp \
    src/pmcalibrationthread.cpp \
  #  src/v4l2uvc.c \
   # src/uvccapture.cpp \ # \
    src/intrcalibthread.cpp \
    src/laserdatathread.cpp \
    src/captureframethread.cpp

# src/intrinsiccameracalibthread.cpp
HEADERS += include/mainwindow.h \
    include/capturethread.h \
    include/calibratethread.h \
    include/cameracalibrator.h \
    include/pmcalibrationthread.h \
   # include/v4l2uvc.h \
   # include/uvccapture.h \ # \
    include/intrcalibthread.h \
    include/laserdatathread.h \
    include/captureframethread.h
