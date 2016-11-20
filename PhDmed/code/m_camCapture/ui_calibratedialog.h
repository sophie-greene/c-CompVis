/********************************************************************************
** Form generated from reading UI file 'calibratedialog.ui'
**
** Created: Tue Jun 11 12:58:03 2013
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
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_calibrateDialog
{
public:
    QPushButton *cmdCancel;
    QLabel *lblMsg;

    void setupUi(QDialog *calibrateDialog)
    {
        if (calibrateDialog->objectName().isEmpty())
            calibrateDialog->setObjectName(QString::fromUtf8("calibrateDialog"));
        calibrateDialog->resize(810, 682);
        cmdCancel = new QPushButton(calibrateDialog);
        cmdCancel->setObjectName(QString::fromUtf8("cmdCancel"));
        cmdCancel->setGeometry(QRect(690, 620, 88, 27));
        lblMsg = new QLabel(calibrateDialog);
        lblMsg->setObjectName(QString::fromUtf8("lblMsg"));
        lblMsg->setGeometry(QRect(60, 50, 621, 16));
        lblMsg->setAutoFillBackground(true);

        retranslateUi(calibrateDialog);

        QMetaObject::connectSlotsByName(calibrateDialog);
    } // setupUi

    void retranslateUi(QDialog *calibrateDialog)
    {
        calibrateDialog->setWindowTitle(QApplication::translate("calibrateDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        cmdCancel->setText(QApplication::translate("calibrateDialog", "Cancel", 0, QApplication::UnicodeUTF8));
        lblMsg->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class calibrateDialog: public Ui_calibrateDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CALIBRATEDIALOG_H
