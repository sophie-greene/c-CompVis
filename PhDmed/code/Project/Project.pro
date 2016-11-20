# -------------------------------------------------
# Project created by QtCreator 2013-05-19T07:43:09
# Sophie Greene
# -------------------------------------------------
QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets
CONFIG += link_pkgconfig
PKGCONFIG += opencv
TARGET = reconstruction
TEMPLATE = app
SOURCES += src/main.cpp \
    src/mcamcap.cpp \
    src/uvccapture.cpp \
    src/v4l2uvc.c \
    src/capturethread.cpp \
    src/calibratedialog.cpp
HEADERS += src/mcamcap.h \
    src/uvccapture.h \
    src/v4l2uvc.h \
    src/capturethread.h \
    src/calibratedialog.h
FORMS += src/mcamcap.ui \
    src/calibratedialog.ui
LIBS += -ljpeg
