QT += core \
    gui
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets
TEMPLATE      = app

HEADERS       = imagewindow.h \
                transactionthread.h
SOURCES       = imagewindow.cpp \
                main.cpp \
                transactionthread.cpp
FORMS         = resizedialog.ui
