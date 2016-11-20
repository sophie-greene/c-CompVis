# -------------------------------------------------
# Project created by QtCreator 2012-12-10T12:43:13
# -------------------------------------------------
TARGET = uvc_cam
TEMPLATE = app
LIBS += -LD:/usr/lib64/glib-1.2 -lglib \
    -LD:/usr/lib64/glib-2.0 -lglib
INCLUDEPATH +=/usr/include/glib-1.2 \
        /usr/include/glib-2.0
SOURCES += main.cpp \
    mainwindow.cpp \
    uvc_camera.cpp \
    jpeg.c \
    colorspaces.c
HEADERS += mainwindow.h \
    uvc_camera.hpp \
    jpeg.h \
    defines.hpp \
    colorspaces.h \
    defs.h \
    huffman.h
FORMS += mainwindow.ui

