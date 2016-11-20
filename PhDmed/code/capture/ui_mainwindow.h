/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Sun May 19 07:33:07 2013
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *btnShow;
    QLabel *lblCam_0;
    QLabel *lblCam_1;
    QLabel *lblCam_2;
    QLabel *lblCam_3;
    QLabel *lblCam_4;
    QLabel *lblCam_5;
    QLabel *lblCam_6;
    QLabel *lblCam_7;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        btnShow = new QPushButton(centralWidget);
        btnShow->setObjectName(QString::fromUtf8("btnShow"));
        btnShow->setGeometry(QRect(390, 510, 88, 27));
        lblCam_0 = new QLabel(centralWidget);
        lblCam_0->setObjectName(QString::fromUtf8("lblCam_0"));
        lblCam_0->setGeometry(QRect(20, 10, 200, 150));
        lblCam_0->setAutoFillBackground(true);
        lblCam_1 = new QLabel(centralWidget);
        lblCam_1->setObjectName(QString::fromUtf8("lblCam_1"));
        lblCam_1->setGeometry(QRect(220, 0, 171, 161));
        lblCam_1->setAutoFillBackground(true);
        lblCam_2 = new QLabel(centralWidget);
        lblCam_2->setObjectName(QString::fromUtf8("lblCam_2"));
        lblCam_2->setGeometry(QRect(400, 0, 171, 161));
        lblCam_2->setAutoFillBackground(true);
        lblCam_3 = new QLabel(centralWidget);
        lblCam_3->setObjectName(QString::fromUtf8("lblCam_3"));
        lblCam_3->setGeometry(QRect(480, -20, 171, 161));
        lblCam_3->setAutoFillBackground(true);
        lblCam_4 = new QLabel(centralWidget);
        lblCam_4->setObjectName(QString::fromUtf8("lblCam_4"));
        lblCam_4->setGeometry(QRect(10, 300, 171, 161));
        lblCam_4->setAutoFillBackground(true);
        lblCam_5 = new QLabel(centralWidget);
        lblCam_5->setObjectName(QString::fromUtf8("lblCam_5"));
        lblCam_5->setGeometry(QRect(210, 280, 171, 161));
        lblCam_5->setAutoFillBackground(true);
        lblCam_6 = new QLabel(centralWidget);
        lblCam_6->setObjectName(QString::fromUtf8("lblCam_6"));
        lblCam_6->setGeometry(QRect(600, 290, 171, 161));
        lblCam_6->setAutoFillBackground(true);
        lblCam_7 = new QLabel(centralWidget);
        lblCam_7->setObjectName(QString::fromUtf8("lblCam_7"));
        lblCam_7->setGeometry(QRect(420, 470, 171, 161));
        lblCam_7->setAutoFillBackground(true);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 800, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        btnShow->setText(QApplication::translate("MainWindow", "Pause", 0, QApplication::UnicodeUTF8));
        lblCam_0->setText(QString());
        lblCam_1->setText(QString());
        lblCam_2->setText(QString());
        lblCam_3->setText(QString());
        lblCam_4->setText(QString());
        lblCam_5->setText(QString());
        lblCam_6->setText(QString());
        lblCam_7->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
