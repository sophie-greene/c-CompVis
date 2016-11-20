QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets
TEMPLATE      = app
CONFIG       += console thread
HEADERS       = thread.h \
                threaddialog.h
SOURCES       = main.cpp \
                thread.cpp \
                threaddialog.cpp
