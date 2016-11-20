# -------------------------------------------------
# Project created by QtCreator 2012-04-16T10:39:22
# -------------------------------------------------
QT -= gui
QT += console sql

#OpenCV
# `pkg-config opencv --libs --cflags`
# -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d
# -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann -lopencv_nonfree
LIBS += /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_contrib.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_gpu.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_legacy.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_nonfree.so /usr/local/lib/libopencv_objdetect.so \
        /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_ts.so /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videostab.so
INCLUDEPATH += /usr/local/include/opencv /usr/local/include

#PCL
LIBS += /usr/lib/libpcl_common.so /usr/lib/libpcl_kdtree.so
INCLUDEPATH += /usr/include/pcl-1.6/

#Eigen
INCLUDEPATH += /usr/include/eigen3/

#SSEImageMatch
INCLUDEPATH += /home/lbaudouin/devel/libSovinSSEImageMatch/

#Matcher
INCLUDEPATH += /home/lbaudouin/devel/include/Matcher/

#SBA
INCLUDEPATH += /home/lbaudouin/devel/sba/

#BA lasmea
INCLUDEPATH += /home/lbaudouin/devel/include/BundleAdjust/

#CERES
INCLUDEPATH += /home/lbaudouin/devel/include/

#OpenMP
LIBS += -fopenmp

TARGET = Reconstrution3D
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
SOURCES += src/reconstruction3D.cpp \
    src/reconstruction3D_simple.cpp \
    src/estimator.cpp \
    bin/build3DfromVideo_simple2.cpp \
    src/detector.cpp \
    src/detector3.cpp \
    src/tools.cpp \
    src/mutualinformation.cpp \
    src/merger.cpp \
    test/MAlgo5Points.cc \
    test/MPolynome.cc \
    test/Matrix33.cc \
    test/matrix.cc \
    test/MathUtils.cc \
    src/reconstruction3D_sovin.cpp \
    test/MReconstruction3VuesModeleUnifie.cc \
    test/MReconstruction3Vues.cc \
    src/bundle_adjustment.cpp \
    src/sba.cpp \
    src/loopclosure.cpp \
    src/mysba.cpp \
    src/omni.cpp \
    src/genericcamera.cpp
HEADERS += include/reconstruction3D.h \
    include/reconstruction3D_simple.h \
    include/estimator.h \
    bin/matlab.h \
    include/detector.h \
    include/detector3.h \
    include/color.h \
    bin/imageprocessing.h \
    bin/fileprocessing.h \
    bin/gnuplot.h \
    include/pointsmanager.h \
    include/mutualinformation.h \
    include/merger.h \
    include/tools.h \
    include/pointsmanager_simple.h \
    test/MAlgo5Points.h \
    test/MPolynome.h \
    test/Matrix33.h \
    test/matrix.h \
    test/MathUtils.h \
    include/reconstruction3D_sovin.h \
    test/MReconstruction3VuesModeleUnifie.h \
    test/MReconstruction3Vues.h \
    include/bundle_adjustment.h \
    include/sba.h \
    include/camera.h \
    include/sba_ceres.h \
    include/BA_lasmea.h \
    include/omni.h \
    include/timer.h \
    include/loopclosure.h \
    include/datamanager.h \
    include/mysba.h \
    include/genericcamera.h
OTHER_FILES += bin/build3Dfrom2views.cpp \
    bin/build3DfromVideo.cpp \
    bin/build3DfromVideo_sovin.cpp \
    bin/calibration.cpp \
    bin/createimages.cpp \
    bin/createimages_simple.cpp \
    bin/simpletest.cpp \
    bin/selectpoints.cpp \
    test/test_icp.cpp \
    bin/movecloud.cpp \
    test/test_merger.cpp \
    test/test_MI.cpp \
    bin/createcloud.cpp \
    bin/splitcloud.cpp \
    test/test_SIFT.cpp \
    bin/unwarp_omni.cpp \
    test/test_matcher.cpp \
    test/test_ReadBdl.cpp \
    bin/build3DfromVideo_simple.cpp \
    test/test_SBA.cpp \
    CMakeLists.txt \
    bin/CMakeLists.txt
