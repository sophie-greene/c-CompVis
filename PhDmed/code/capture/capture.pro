# -------------------------------------------------
# Project created by QtCreator 2012-12-19T14:30:03
# -------------------------------------------------
TARGET = capture
TEMPLATE = app
CONFIG += link_pkgconfig
PKGCONFIG += opencv
SOURCES += main.cpp \
    mainwindow.cpp \
    v4l2uvc.c \
    uvccapture.cpp \
  #  capturethread.cpp \
    capturevideo.cpp
HEADERS += mainwindow.h \
    v4l2uvc.h \
    uvccapture.h \
    #capturethread.h \
    capturevideo.h
FORMS += mainwindow.ui
LIBS += -ljpeg
