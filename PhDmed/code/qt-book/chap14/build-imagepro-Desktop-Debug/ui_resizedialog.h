/********************************************************************************
** Form generated from reading UI file 'resizedialog.ui'
**
** Created by: Qt User Interface Compiler version 5.0.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RESIZEDIALOG_H
#define UI_RESIZEDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ResizeDialog
{
public:
    QVBoxLayout *vboxLayout;
    QLabel *mainLabel;
    QHBoxLayout *hboxLayout;
    QSpinBox *widthSpinBox;
    QLabel *xLabel;
    QSpinBox *heightSpinBox;
    QHBoxLayout *hboxLayout1;
    QSpacerItem *spacerItem;
    QPushButton *okButton;
    QPushButton *cancelButton;

    void setupUi(QDialog *ResizeDialog)
    {
        if (ResizeDialog->objectName().isEmpty())
            ResizeDialog->setObjectName(QStringLiteral("ResizeDialog"));
        ResizeDialog->resize(190, 129);
        vboxLayout = new QVBoxLayout(ResizeDialog);
        vboxLayout->setObjectName(QStringLiteral("vboxLayout"));
        mainLabel = new QLabel(ResizeDialog);
        mainLabel->setObjectName(QStringLiteral("mainLabel"));

        vboxLayout->addWidget(mainLabel);

        hboxLayout = new QHBoxLayout();
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        widthSpinBox = new QSpinBox(ResizeDialog);
        widthSpinBox->setObjectName(QStringLiteral("widthSpinBox"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widthSpinBox->sizePolicy().hasHeightForWidth());
        widthSpinBox->setSizePolicy(sizePolicy);
        widthSpinBox->setMaximum(10000);

        hboxLayout->addWidget(widthSpinBox);

        xLabel = new QLabel(ResizeDialog);
        xLabel->setObjectName(QStringLiteral("xLabel"));
        xLabel->setAlignment(Qt::AlignCenter);

        hboxLayout->addWidget(xLabel);

        heightSpinBox = new QSpinBox(ResizeDialog);
        heightSpinBox->setObjectName(QStringLiteral("heightSpinBox"));
        sizePolicy.setHeightForWidth(heightSpinBox->sizePolicy().hasHeightForWidth());
        heightSpinBox->setSizePolicy(sizePolicy);
        heightSpinBox->setMaximum(10000);

        hboxLayout->addWidget(heightSpinBox);


        vboxLayout->addLayout(hboxLayout);

        hboxLayout1 = new QHBoxLayout();
        hboxLayout1->setObjectName(QStringLiteral("hboxLayout1"));
        spacerItem = new QSpacerItem(0, 21, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout1->addItem(spacerItem);

        okButton = new QPushButton(ResizeDialog);
        okButton->setObjectName(QStringLiteral("okButton"));
        okButton->setDefault(true);

        hboxLayout1->addWidget(okButton);

        cancelButton = new QPushButton(ResizeDialog);
        cancelButton->setObjectName(QStringLiteral("cancelButton"));

        hboxLayout1->addWidget(cancelButton);


        vboxLayout->addLayout(hboxLayout1);


        retranslateUi(ResizeDialog);
        QObject::connect(okButton, SIGNAL(clicked()), ResizeDialog, SLOT(accept()));
        QObject::connect(cancelButton, SIGNAL(clicked()), ResizeDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(ResizeDialog);
    } // setupUi

    void retranslateUi(QDialog *ResizeDialog)
    {
        ResizeDialog->setWindowTitle(QApplication::translate("ResizeDialog", "Image Pro", 0));
        mainLabel->setText(QApplication::translate("ResizeDialog", "Enter new size:", 0));
        xLabel->setText(QApplication::translate("ResizeDialog", "x", 0));
        okButton->setText(QApplication::translate("ResizeDialog", "OK", 0));
        cancelButton->setText(QApplication::translate("ResizeDialog", "Cancel", 0));
    } // retranslateUi

};

namespace Ui {
    class ResizeDialog: public Ui_ResizeDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RESIZEDIALOG_H
