QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgetsTARGET = reconstruction

TEMPLATE = app

CONFIG += link_pkgconfig

PKGCONFIG += opencv

SOURCES += src/main.cpp \
    src/mainwindow.cpp \
    src/capturethread.cpp \
    src/calibratethread.cpp \
    src/cameracalibrator.cpp

HEADERS += src/mainwindow.h \
    src/capturethread.h \
    src/calibratethread.h \
    src/cameracalibrator.h
