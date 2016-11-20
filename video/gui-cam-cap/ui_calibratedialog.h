/********************************************************************************
** Form generated from reading UI file 'calibratedialog.ui'
**
** Created: Thu Jun 13 19:02:31 2013
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CALIBRATEDIALOG_H
#define UI_CALIBRATEDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>

QT_BEGIN_NAMESPACE

class Ui_CalibrateDialog
{
public:
    QLabel *lblMsg;
    QLabel *lblCam_3;
    QLabel *lblCam_4;
    QLabel *label_7;
    QLabel *lblCam_1;
    QLabel *lblCam_5;
    QLabel *lblCam_6;
    QLabel *label_8;
    QLabel *lblCam_0;
    QLabel *label_3;
    QLabel *lblCam_2;
    QLabel *lblCam_7;
    QLabel *label;
    QLabel *label_6;
    QLabel *label_2;
    QLabel *label_4;
    QLabel *label_5;

    void setupUi(QDialog *CalibrateDialog)
    {
        if (CalibrateDialog->objectName().isEmpty())
            CalibrateDialog->setObjectName(QString::fromUtf8("CalibrateDialog"));
        CalibrateDialog->resize(718, 611);
        lblMsg = new QLabel(CalibrateDialog);
        lblMsg->setObjectName(QString::fromUtf8("lblMsg"));
        lblMsg->setGeometry(QRect(50, 20, 571, 17));
        lblCam_3 = new QLabel(CalibrateDialog);
        lblCam_3->setObjectName(QString::fromUtf8("lblCam_3"));
        lblCam_3->setGeometry(QRect(10, 270, 200, 150));
        lblCam_3->setAutoFillBackground(true);
        lblCam_3->setScaledContents(true);
        lblCam_4 = new QLabel(CalibrateDialog);
        lblCam_4->setObjectName(QString::fromUtf8("lblCam_4"));
        lblCam_4->setGeometry(QRect(240, 270, 200, 150));
        lblCam_4->setAutoFillBackground(true);
        lblCam_4->setScaledContents(true);
        label_7 = new QLabel(CalibrateDialog);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 430, 220, 17));
        lblCam_1 = new QLabel(CalibrateDialog);
        lblCam_1->setObjectName(QString::fromUtf8("lblCam_1"));
        lblCam_1->setGeometry(QRect(240, 90, 200, 150));
        lblCam_1->setAutoFillBackground(true);
        lblCam_1->setScaledContents(true);
        lblCam_5 = new QLabel(CalibrateDialog);
        lblCam_5->setObjectName(QString::fromUtf8("lblCam_5"));
        lblCam_5->setGeometry(QRect(470, 270, 200, 150));
        lblCam_5->setAutoFillBackground(true);
        lblCam_5->setScaledContents(true);
        lblCam_6 = new QLabel(CalibrateDialog);
        lblCam_6->setObjectName(QString::fromUtf8("lblCam_6"));
        lblCam_6->setGeometry(QRect(10, 450, 200, 150));
        lblCam_6->setAutoFillBackground(true);
        lblCam_6->setScaledContents(true);
        label_8 = new QLabel(CalibrateDialog);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(240, 430, 220, 17));
        lblCam_0 = new QLabel(CalibrateDialog);
        lblCam_0->setObjectName(QString::fromUtf8("lblCam_0"));
        lblCam_0->setGeometry(QRect(10, 90, 300, 150));
        lblCam_0->setAutoFillBackground(true);
        lblCam_0->setScaledContents(true);
        label_3 = new QLabel(CalibrateDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(470, 70, 220, 17));
        lblCam_2 = new QLabel(CalibrateDialog);
        lblCam_2->setObjectName(QString::fromUtf8("lblCam_2"));
        lblCam_2->setGeometry(QRect(470, 90, 200, 150));
        lblCam_2->setAutoFillBackground(true);
        lblCam_2->setScaledContents(true);
        lblCam_7 = new QLabel(CalibrateDialog);
        lblCam_7->setObjectName(QString::fromUtf8("lblCam_7"));
        lblCam_7->setGeometry(QRect(240, 450, 200, 150));
        lblCam_7->setAutoFillBackground(true);
        lblCam_7->setScaledContents(true);
        label = new QLabel(CalibrateDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 70, 220, 17));
        label->setScaledContents(true);
        label_6 = new QLabel(CalibrateDialog);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(470, 250, 220, 17));
        label_2 = new QLabel(CalibrateDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(370, 70, 220, 17));
        label_4 = new QLabel(CalibrateDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 250, 220, 17));
        label_5 = new QLabel(CalibrateDialog);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(240, 250, 220, 17));

        retranslateUi(CalibrateDialog);

        QMetaObject::connectSlotsByName(CalibrateDialog);
    } // setupUi

    void retranslateUi(QDialog *CalibrateDialog)
    {
        CalibrateDialog->setWindowTitle(QApplication::translate("CalibrateDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        lblMsg->setText(QApplication::translate("CalibrateDialog", "Calibrating........ ", 0, QApplication::UnicodeUTF8));
        lblCam_3->setText(QString());
        lblCam_4->setText(QString());
        label_7->setText(QString());
        lblCam_1->setText(QString());
        lblCam_5->setText(QString());
        lblCam_6->setText(QString());
        label_8->setText(QString());
        lblCam_0->setText(QString());
        label_3->setText(QString());
        lblCam_2->setText(QString());
        lblCam_7->setText(QString());
        label->setText(QString());
        label_6->setText(QString());
        label_2->setText(QString());
        label_4->setText(QString());
        label_5->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class CalibrateDialog: public Ui_CalibrateDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CALIBRATEDIALOG_H
