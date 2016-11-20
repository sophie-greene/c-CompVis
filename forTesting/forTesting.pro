#-------------------------------------------------
#
# Project created by QtCreator 2013-07-02T03:05:57
#
#-------------------------------------------------

TARGET = forTesting
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
