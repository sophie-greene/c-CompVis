#-------------------------------------------------
#
# Project created by QtCreator 2013-08-21T10:49:27
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Sophie
CONFIG   += console
CONFIG   -= app_bundle
CONFIG += link_pkgconfig
PKGCONFIG += opencv \
              glib-2.0

TEMPLATE = app


SOURCES += main.cpp \
    #wingedge.cpp \
    #visualinfo.c \
    #visual_app.cpp \
  # video.cpp \
    uvc_camera.cpp\
   # tinyxmlparser.cpp \
    #tinyxmlerror.cpp \
    #tinyxml.cpp \
    #tinystr.cpp \
  # texture.cpp \
  #  state.cpp \
   # shapes.cpp \
    #shader.cpp \
   # s9xml.cpp \
   # primitive.cpp \
  jpeg.c \
   # glfw_app.cpp \
   # glewinfo.c \
    #glew.c \
    #fbo.cpp \
    #cvprocess.cpp \
    colorspaces.c #\
    #camera.cpp \
  #  asset.cpp

    #uvc_camera.cpp

INCLUDEPATH += /usr/not-backed-up/OpenCV-2.4-/include
INCLUDEPATH += /usr/not-backed-up/OpenCV-2.4-/include/opencv
INCLUDEPATH += /usr/not-backed-up/OpenCV-2.4-/include/opencv2
#INCLUDEPATH += /usr/include/glib-2.0
#HEADERS += \
   # uvc_camera.hpp

LIBS+=-lboost_thread-mt \
        -ljpeg \
   #   -lglib   # \
-lglut\
#-lglfw \
-lGL

     #   -ls9gear


HEADERS += \
    #wingedge.hpp \
    #visualapp.hpp \
   #video.hpp \
    #vertex_types.hpp \
    uvc_camera.hpp \
   # utils.hpp \
    #tinyxml.h \
    #tinystr.h \
   # texture.hpp \
    #state.hpp \
   # shapes.hpp \
    #shader.hpp \
    #s9xml.hpp \
   # primitive.hpp \
  #  main.cpp.autosave \
   jpeg.h \
    #image.hpp \
    #huffman.h \
    #gtk_functions.hpp \
    #glfw_app.hpp \
    #glasset.hpp \
    #geometry.hpp \
    #fbo.hpp \
    #events.hpp \
   #defs.h \
  #  defines.hpp \
   # cvprocess.hpp \
   # common.hpp \
   colorspaces.h #\
    #camera.hpp \
  #  asset.hpp \
    #com/common.hpp
