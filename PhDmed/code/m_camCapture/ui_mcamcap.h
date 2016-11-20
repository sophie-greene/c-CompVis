/********************************************************************************
** Form generated from reading UI file 'mcamcap.ui'
**
** Created: Tue Jun 11 12:24:40 2013
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MCAMCAP_H
#define UI_MCAMCAP_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_mCamCap
{
public:
    QWidget *centralWidget;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *cmdCalibrate;
    QPushButton *cmdCapture;
    QSpacerItem *horizontalSpacer;
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

    void setupUi(QMainWindow *mCamCap)
    {
        if (mCamCap->objectName().isEmpty())
            mCamCap->setObjectName(QString::fromUtf8("mCamCap"));
        mCamCap->resize(810, 682);
        centralWidget = new QWidget(mCamCap);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        horizontalLayoutWidget_2 = new QWidget(centralWidget);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(440, 610, 351, 29));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        cmdCalibrate = new QPushButton(horizontalLayoutWidget_2);
        cmdCalibrate->setObjectName(QString::fromUtf8("cmdCalibrate"));

        horizontalLayout_2->addWidget(cmdCalibrate);

        cmdCapture = new QPushButton(horizontalLayoutWidget_2);
        cmdCapture->setObjectName(QString::fromUtf8("cmdCapture"));

        horizontalLayout_2->addWidget(cmdCapture);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        lblCam_0 = new QLabel(centralWidget);
        lblCam_0->setObjectName(QString::fromUtf8("lblCam_0"));
        lblCam_0->setGeometry(QRect(60, 40, 121, 101));
        lblCam_0->setAutoFillBackground(true);
        lblCam_1 = new QLabel(centralWidget);
        lblCam_1->setObjectName(QString::fromUtf8("lblCam_1"));
        lblCam_1->setGeometry(QRect(240, 30, 121, 101));
        lblCam_1->setAutoFillBackground(true);
        lblCam_2 = new QLabel(centralWidget);
        lblCam_2->setObjectName(QString::fromUtf8("lblCam_2"));
        lblCam_2->setGeometry(QRect(440, 30, 121, 101));
        lblCam_2->setAutoFillBackground(true);
        lblCam_3 = new QLabel(centralWidget);
        lblCam_3->setObjectName(QString::fromUtf8("lblCam_3"));
        lblCam_3->setGeometry(QRect(60, 190, 121, 101));
        lblCam_3->setAutoFillBackground(true);
        lblCam_4 = new QLabel(centralWidget);
        lblCam_4->setObjectName(QString::fromUtf8("lblCam_4"));
        lblCam_4->setGeometry(QRect(250, 180, 121, 101));
        lblCam_4->setAutoFillBackground(true);
        lblCam_5 = new QLabel(centralWidget);
        lblCam_5->setObjectName(QString::fromUtf8("lblCam_5"));
        lblCam_5->setGeometry(QRect(410, 180, 121, 101));
        lblCam_5->setAutoFillBackground(true);
        lblCam_6 = new QLabel(centralWidget);
        lblCam_6->setObjectName(QString::fromUtf8("lblCam_6"));
        lblCam_6->setGeometry(QRect(40, 340, 121, 101));
        lblCam_6->setAutoFillBackground(true);
        lblCam_7 = new QLabel(centralWidget);
        lblCam_7->setObjectName(QString::fromUtf8("lblCam_7"));
        lblCam_7->setGeometry(QRect(210, 330, 121, 101));
        lblCam_7->setAutoFillBackground(true);
        mCamCap->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(mCamCap);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 810, 25));
        menuBar->setAutoFillBackground(true);
        mCamCap->setMenuBar(menuBar);
        mainToolBar = new QToolBar(mCamCap);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        mCamCap->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(mCamCap);

        QMetaObject::connectSlotsByName(mCamCap);
    } // setupUi

    void retranslateUi(QMainWindow *mCamCap)
    {
        mCamCap->setWindowTitle(QApplication::translate("mCamCap", "Capture Multiple Cameras", 0, QApplication::UnicodeUTF8));
        cmdCalibrate->setText(QApplication::translate("mCamCap", "Calibrate", 0, QApplication::UnicodeUTF8));
        cmdCapture->setText(QApplication::translate("mCamCap", "Capture", 0, QApplication::UnicodeUTF8));
        lblCam_0->setText(QApplication::translate("mCamCap", "Cam0", 0, QApplication::UnicodeUTF8));
        lblCam_1->setText(QApplication::translate("mCamCap", "Cam1", 0, QApplication::UnicodeUTF8));
        lblCam_2->setText(QApplication::translate("mCamCap", "Cam2", 0, QApplication::UnicodeUTF8));
        lblCam_3->setText(QApplication::translate("mCamCap", "Cam3", 0, QApplication::UnicodeUTF8));
        lblCam_4->setText(QApplication::translate("mCamCap", "Cam4", 0, QApplication::UnicodeUTF8));
        lblCam_5->setText(QApplication::translate("mCamCap", "Cam5", 0, QApplication::UnicodeUTF8));
        lblCam_6->setText(QApplication::translate("mCamCap", "Cam6", 0, QApplication::UnicodeUTF8));
        lblCam_7->setText(QApplication::translate("mCamCap", "Cam7 hello", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class mCamCap: public Ui_mCamCap {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MCAMCAP_H
