# -------------------------------------------------
# Project created by QtCreator 2013-05-19T07:43:09
# Sophie Greene
# -------------------------------------------------
QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets
CONFIG += link_pkgconfig
PKGCONFIG += opencv
TARGET = m_camCapture
TEMPLATE = app
SOURCES += src/main.cpp \
    src/mcamcap.cpp \
    src/uvccapture.cpp \
    src/v4l2uvc.c \
    src/capturethread.cpp \
    src/calibratedialog.cpp \
    src/calibratethread.cpp \
    cameracalibrator.cpp
HEADERS += src/mcamcap.h \
    src/uvccapture.h \
    src/v4l2uvc.h \
    src/capturethread.h \
    src/calibratedialog.h \
    src/calibratethread.h \
    cameracalibrator.h
FORMS += src/mcamcap.ui \
    src/calibratedialog.ui
LIBS += -ljpeg
