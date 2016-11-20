QT += core \
    gui
TARGET = qt-opencv-multithreaded
TEMPLATE = app
VERSION = 1.21
DEFINES += APP_VERSION=$$VERSION
FORMS = ImageProcessingSettingsDialog.ui \
    CameraConnectDialog.ui \
    MainWindow.ui
CONFIG += link_pkgconfig
PKGCONFIG += opencv
SOURCES += main.cpp \
    MainWindow.cpp \
    CaptureThread.cpp \
    Controller.cpp \
    ImageBuffer.cpp \
    CameraConnectDialog.cpp \
    ProcessingThread.cpp \
    FrameLabel.cpp \
    MatToQImage.cpp \
    ImageProcessingSettingsDialog.cpp \
    v4l2uvc.c \
    uvccapture.cpp
HEADERS += MainWindow.h \
    CaptureThread.h \
    Controller.h \
    ImageBuffer.h \
    CameraConnectDialog.h \
    ProcessingThread.h \
    FrameLabel.h \
    Structures.h \
    Config.h \
    MatToQImage.h \
    ImageProcessingSettingsDialog.h \
    v4l2uvc.h \
    uvccapture.h
