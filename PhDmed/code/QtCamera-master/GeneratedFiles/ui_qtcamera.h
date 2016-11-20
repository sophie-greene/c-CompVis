/********************************************************************************
** Form generated from reading UI file 'qtcamera.ui'
**
** Created: Fri Dec 7 16:34:45 2012
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QTCAMERA_H
#define UI_QTCAMERA_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QtCameraClass
{
public:
    QAction *actionExit;
    QAction *actionStart;
    QAction *actionStop;
    QAction *actionScale;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuCamera;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QtCameraClass)
    {
        if (QtCameraClass->objectName().isEmpty())
            QtCameraClass->setObjectName(QString::fromUtf8("QtCameraClass"));
        QtCameraClass->resize(658, 562);
        actionExit = new QAction(QtCameraClass);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionStart = new QAction(QtCameraClass);
        actionStart->setObjectName(QString::fromUtf8("actionStart"));
        actionStart->setIconVisibleInMenu(false);
        actionStop = new QAction(QtCameraClass);
        actionStop->setObjectName(QString::fromUtf8("actionStop"));
        actionStop->setIconVisibleInMenu(false);
        actionScale = new QAction(QtCameraClass);
        actionScale->setObjectName(QString::fromUtf8("actionScale"));
        actionScale->setCheckable(true);
        actionScale->setChecked(true);
        centralWidget = new QWidget(QtCameraClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        QtCameraClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QtCameraClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 658, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuCamera = new QMenu(menuBar);
        menuCamera->setObjectName(QString::fromUtf8("menuCamera"));
        QtCameraClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(QtCameraClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        QtCameraClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(QtCameraClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        QtCameraClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuCamera->menuAction());
        menuFile->addAction(actionExit);
        menuCamera->addAction(actionStart);
        menuCamera->addAction(actionStop);
        menuCamera->addSeparator();
        menuCamera->addAction(actionScale);
        mainToolBar->addAction(actionStart);
        mainToolBar->addAction(actionStop);

        retranslateUi(QtCameraClass);

        QMetaObject::connectSlotsByName(QtCameraClass);
    } // setupUi

    void retranslateUi(QMainWindow *QtCameraClass)
    {
        QtCameraClass->setWindowTitle(QApplication::translate("QtCameraClass", "QtCamera", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("QtCameraClass", "Exit", 0, QApplication::UnicodeUTF8));
        actionStart->setText(QApplication::translate("QtCameraClass", "Start", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionStart->setToolTip(QApplication::translate("QtCameraClass", "Start streaming video", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionStop->setText(QApplication::translate("QtCameraClass", "Stop", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionStop->setToolTip(QApplication::translate("QtCameraClass", "Stop video streaming", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionScale->setText(QApplication::translate("QtCameraClass", "Scaling", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("QtCameraClass", "File", 0, QApplication::UnicodeUTF8));
        menuCamera->setTitle(QApplication::translate("QtCameraClass", "Camera", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class QtCameraClass: public Ui_QtCameraClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QTCAMERA_H
