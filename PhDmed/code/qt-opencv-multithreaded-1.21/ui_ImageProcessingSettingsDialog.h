/********************************************************************************
** Form generated from reading UI file 'ImageProcessingSettingsDialog.ui'
**
** Created: Fri Dec 7 10:21:54 2012
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMAGEPROCESSINGSETTINGSDIALOG_H
#define UI_IMAGEPROCESSINGSETTINGSDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ImageProcessingSettingsDialog
{
public:
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_1;
    QTabWidget *tabWidget;
    QWidget *smoothTab;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_2;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *smoothBlurButton;
    QRadioButton *smoothGaussianButton;
    QRadioButton *smoothMedianButton;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_3;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QLineEdit *smoothParam1Edit;
    QLabel *smoothParam1RangeLabel;
    QLabel *smoothParam1Label;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_5;
    QLineEdit *smoothParam2Edit;
    QLabel *smoothParam2RangeLabel;
    QLabel *smoothParam2Label;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_6;
    QLineEdit *smoothParam3Edit;
    QLabel *smoothParam3RangeLabel;
    QLabel *smoothParam3Label;
    QHBoxLayout *horizontalLayout_1;
    QLabel *label_7;
    QLineEdit *smoothParam4Edit;
    QLabel *smoothParam4RangeLabel;
    QLabel *smoothParam4Label;
    QSpacerItem *verticalSpacer_2;
    QSpacerItem *verticalSpacer_7;
    QPushButton *resetSmoothToDefaultsButton;
    QWidget *dilateTab;
    QWidget *layoutWidget3;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_14;
    QLineEdit *dilateIterationsEdit;
    QLabel *label;
    QSpacerItem *verticalSpacer_3;
    QPushButton *resetDilateToDefaultsButton;
    QWidget *erodeTab;
    QWidget *layoutWidget4;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_17;
    QLineEdit *erodeIterationsEdit;
    QLabel *label_13;
    QSpacerItem *verticalSpacer_4;
    QPushButton *resetErodeToDefaultsButton;
    QWidget *flipTab;
    QWidget *layoutWidget5;
    QVBoxLayout *verticalLayout_9;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_12;
    QRadioButton *flipXAxisButton;
    QRadioButton *flipYAxisButton;
    QRadioButton *flipBothAxesButton;
    QSpacerItem *verticalSpacer_5;
    QPushButton *resetFlipToDefaultsButton;
    QWidget *cannyTab;
    QWidget *layoutWidget6;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_1;
    QLineEdit *cannyThresh1Edit;
    QLabel *label_8;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_16;
    QLineEdit *cannyThresh2Edit;
    QLabel *label_20;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_10;
    QLineEdit *cannyApertureSizeEdit;
    QLabel *label_9;
    QCheckBox *cannyL2NormCheckBox;
    QSpacerItem *verticalSpacer_6;
    QPushButton *resetCannyToDefaultsButton;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *applyButton;
    QPushButton *resetAllToDefaultsButton;
    QFrame *line_1;
    QDialogButtonBox *okCancelBox;
    QButtonGroup *flipCodeGroup;
    QButtonGroup *smoothTypeGroup;

    void setupUi(QDialog *ImageProcessingSettingsDialog)
    {
        if (ImageProcessingSettingsDialog->objectName().isEmpty())
            ImageProcessingSettingsDialog->setObjectName(QString::fromUtf8("ImageProcessingSettingsDialog"));
        ImageProcessingSettingsDialog->resize(441, 381);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ImageProcessingSettingsDialog->sizePolicy().hasHeightForWidth());
        ImageProcessingSettingsDialog->setSizePolicy(sizePolicy);
        ImageProcessingSettingsDialog->setMinimumSize(QSize(0, 0));
        layoutWidget1 = new QWidget(ImageProcessingSettingsDialog);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(11, 11, 421, 361));
        verticalLayout_1 = new QVBoxLayout(layoutWidget1);
        verticalLayout_1->setObjectName(QString::fromUtf8("verticalLayout_1"));
        verticalLayout_1->setContentsMargins(0, 0, 0, 0);
        tabWidget = new QTabWidget(layoutWidget1);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        smoothTab = new QWidget();
        smoothTab->setObjectName(QString::fromUtf8("smoothTab"));
        layoutWidget2 = new QWidget(smoothTab);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_5 = new QVBoxLayout(layoutWidget2);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_2 = new QLabel(layoutWidget2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy1);
        label_2->setMinimumSize(QSize(0, 27));
        label_2->setMaximumSize(QSize(140, 27));
        QFont font;
        font.setPointSize(8);
        font.setBold(true);
        font.setWeight(75);
        label_2->setFont(font);

        verticalLayout_3->addWidget(label_2);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        smoothBlurButton = new QRadioButton(layoutWidget2);
        smoothTypeGroup = new QButtonGroup(ImageProcessingSettingsDialog);
        smoothTypeGroup->setObjectName(QString::fromUtf8("smoothTypeGroup"));
        smoothTypeGroup->addButton(smoothBlurButton);
        smoothBlurButton->setObjectName(QString::fromUtf8("smoothBlurButton"));
        smoothBlurButton->setMaximumSize(QSize(140, 16777215));
        QFont font1;
        font1.setPointSize(8);
        smoothBlurButton->setFont(font1);

        verticalLayout_2->addWidget(smoothBlurButton);

        smoothGaussianButton = new QRadioButton(layoutWidget2);
        smoothTypeGroup->addButton(smoothGaussianButton);
        smoothGaussianButton->setObjectName(QString::fromUtf8("smoothGaussianButton"));
        smoothGaussianButton->setMaximumSize(QSize(140, 16777215));
        smoothGaussianButton->setFont(font1);

        verticalLayout_2->addWidget(smoothGaussianButton);

        smoothMedianButton = new QRadioButton(layoutWidget2);
        smoothTypeGroup->addButton(smoothMedianButton);
        smoothMedianButton->setObjectName(QString::fromUtf8("smoothMedianButton"));
        smoothMedianButton->setMaximumSize(QSize(140, 16777215));
        smoothMedianButton->setFont(font1);

        verticalLayout_2->addWidget(smoothMedianButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        verticalLayout_3->addLayout(verticalLayout_2);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_3 = new QLabel(layoutWidget2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(0, 27));
        label_3->setMaximumSize(QSize(16777215, 27));
        label_3->setFont(font);

        verticalLayout_4->addWidget(label_3);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(layoutWidget2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy2);
        label_4->setMinimumSize(QSize(15, 0));
        label_4->setMaximumSize(QSize(15, 16777215));
        label_4->setFont(font1);

        horizontalLayout_4->addWidget(label_4);

        smoothParam1Edit = new QLineEdit(layoutWidget2);
        smoothParam1Edit->setObjectName(QString::fromUtf8("smoothParam1Edit"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(smoothParam1Edit->sizePolicy().hasHeightForWidth());
        smoothParam1Edit->setSizePolicy(sizePolicy3);
        smoothParam1Edit->setMinimumSize(QSize(45, 0));
        smoothParam1Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam1Edit->setFont(font1);

        horizontalLayout_4->addWidget(smoothParam1Edit);

        smoothParam1RangeLabel = new QLabel(layoutWidget2);
        smoothParam1RangeLabel->setObjectName(QString::fromUtf8("smoothParam1RangeLabel"));
        smoothParam1RangeLabel->setFont(font);

        horizontalLayout_4->addWidget(smoothParam1RangeLabel);

        smoothParam1Label = new QLabel(layoutWidget2);
        smoothParam1Label->setObjectName(QString::fromUtf8("smoothParam1Label"));
        smoothParam1Label->setFont(font1);

        horizontalLayout_4->addWidget(smoothParam1Label);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_5 = new QLabel(layoutWidget2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        sizePolicy2.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy2);
        label_5->setMinimumSize(QSize(15, 0));
        label_5->setMaximumSize(QSize(15, 16777215));
        label_5->setFont(font1);

        horizontalLayout_3->addWidget(label_5);

        smoothParam2Edit = new QLineEdit(layoutWidget2);
        smoothParam2Edit->setObjectName(QString::fromUtf8("smoothParam2Edit"));
        sizePolicy3.setHeightForWidth(smoothParam2Edit->sizePolicy().hasHeightForWidth());
        smoothParam2Edit->setSizePolicy(sizePolicy3);
        smoothParam2Edit->setMinimumSize(QSize(45, 0));
        smoothParam2Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam2Edit->setFont(font1);

        horizontalLayout_3->addWidget(smoothParam2Edit);

        smoothParam2RangeLabel = new QLabel(layoutWidget2);
        smoothParam2RangeLabel->setObjectName(QString::fromUtf8("smoothParam2RangeLabel"));
        smoothParam2RangeLabel->setFont(font);

        horizontalLayout_3->addWidget(smoothParam2RangeLabel);

        smoothParam2Label = new QLabel(layoutWidget2);
        smoothParam2Label->setObjectName(QString::fromUtf8("smoothParam2Label"));
        smoothParam2Label->setFont(font1);

        horizontalLayout_3->addWidget(smoothParam2Label);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_6 = new QLabel(layoutWidget2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        sizePolicy2.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy2);
        label_6->setMinimumSize(QSize(15, 0));
        label_6->setMaximumSize(QSize(15, 16777215));
        label_6->setFont(font1);

        horizontalLayout_2->addWidget(label_6);

        smoothParam3Edit = new QLineEdit(layoutWidget2);
        smoothParam3Edit->setObjectName(QString::fromUtf8("smoothParam3Edit"));
        sizePolicy3.setHeightForWidth(smoothParam3Edit->sizePolicy().hasHeightForWidth());
        smoothParam3Edit->setSizePolicy(sizePolicy3);
        smoothParam3Edit->setMinimumSize(QSize(45, 0));
        smoothParam3Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam3Edit->setFont(font1);

        horizontalLayout_2->addWidget(smoothParam3Edit);

        smoothParam3RangeLabel = new QLabel(layoutWidget2);
        smoothParam3RangeLabel->setObjectName(QString::fromUtf8("smoothParam3RangeLabel"));
        smoothParam3RangeLabel->setFont(font);

        horizontalLayout_2->addWidget(smoothParam3RangeLabel);

        smoothParam3Label = new QLabel(layoutWidget2);
        smoothParam3Label->setObjectName(QString::fromUtf8("smoothParam3Label"));
        smoothParam3Label->setFont(font1);

        horizontalLayout_2->addWidget(smoothParam3Label);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_1 = new QHBoxLayout();
        horizontalLayout_1->setObjectName(QString::fromUtf8("horizontalLayout_1"));
        label_7 = new QLabel(layoutWidget2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        sizePolicy2.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy2);
        label_7->setMinimumSize(QSize(15, 0));
        label_7->setMaximumSize(QSize(15, 16777215));
        label_7->setFont(font1);

        horizontalLayout_1->addWidget(label_7);

        smoothParam4Edit = new QLineEdit(layoutWidget2);
        smoothParam4Edit->setObjectName(QString::fromUtf8("smoothParam4Edit"));
        sizePolicy3.setHeightForWidth(smoothParam4Edit->sizePolicy().hasHeightForWidth());
        smoothParam4Edit->setSizePolicy(sizePolicy3);
        smoothParam4Edit->setMinimumSize(QSize(45, 0));
        smoothParam4Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam4Edit->setFont(font1);

        horizontalLayout_1->addWidget(smoothParam4Edit);

        smoothParam4RangeLabel = new QLabel(layoutWidget2);
        smoothParam4RangeLabel->setObjectName(QString::fromUtf8("smoothParam4RangeLabel"));
        smoothParam4RangeLabel->setFont(font);

        horizontalLayout_1->addWidget(smoothParam4RangeLabel);

        smoothParam4Label = new QLabel(layoutWidget2);
        smoothParam4Label->setObjectName(QString::fromUtf8("smoothParam4Label"));
        smoothParam4Label->setFont(font1);

        horizontalLayout_1->addWidget(smoothParam4Label);


        verticalLayout->addLayout(horizontalLayout_1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);


        verticalLayout_4->addLayout(verticalLayout);


        horizontalLayout->addLayout(verticalLayout_4);


        verticalLayout_5->addLayout(horizontalLayout);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_7);

        resetSmoothToDefaultsButton = new QPushButton(layoutWidget2);
        resetSmoothToDefaultsButton->setObjectName(QString::fromUtf8("resetSmoothToDefaultsButton"));

        verticalLayout_5->addWidget(resetSmoothToDefaultsButton);

        tabWidget->addTab(smoothTab, QString());
        dilateTab = new QWidget();
        dilateTab->setObjectName(QString::fromUtf8("dilateTab"));
        layoutWidget3 = new QWidget(dilateTab);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_7 = new QVBoxLayout(layoutWidget3);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_14 = new QLabel(layoutWidget3);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setMinimumSize(QSize(0, 27));
        label_14->setFont(font);

        horizontalLayout_7->addWidget(label_14);

        dilateIterationsEdit = new QLineEdit(layoutWidget3);
        dilateIterationsEdit->setObjectName(QString::fromUtf8("dilateIterationsEdit"));
        sizePolicy3.setHeightForWidth(dilateIterationsEdit->sizePolicy().hasHeightForWidth());
        dilateIterationsEdit->setSizePolicy(sizePolicy3);
        dilateIterationsEdit->setMinimumSize(QSize(50, 27));
        dilateIterationsEdit->setMaximumSize(QSize(50, 16777215));
        dilateIterationsEdit->setFont(font1);

        horizontalLayout_7->addWidget(dilateIterationsEdit);

        label = new QLabel(layoutWidget3);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(0, 27));
        label->setFont(font);

        horizontalLayout_7->addWidget(label);


        verticalLayout_7->addLayout(horizontalLayout_7);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer_3);

        resetDilateToDefaultsButton = new QPushButton(layoutWidget3);
        resetDilateToDefaultsButton->setObjectName(QString::fromUtf8("resetDilateToDefaultsButton"));

        verticalLayout_7->addWidget(resetDilateToDefaultsButton);

        tabWidget->addTab(dilateTab, QString());
        erodeTab = new QWidget();
        erodeTab->setObjectName(QString::fromUtf8("erodeTab"));
        layoutWidget4 = new QWidget(erodeTab);
        layoutWidget4->setObjectName(QString::fromUtf8("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_8 = new QVBoxLayout(layoutWidget4);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_17 = new QLabel(layoutWidget4);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setMinimumSize(QSize(0, 27));
        label_17->setFont(font);

        horizontalLayout_8->addWidget(label_17);

        erodeIterationsEdit = new QLineEdit(layoutWidget4);
        erodeIterationsEdit->setObjectName(QString::fromUtf8("erodeIterationsEdit"));
        sizePolicy3.setHeightForWidth(erodeIterationsEdit->sizePolicy().hasHeightForWidth());
        erodeIterationsEdit->setSizePolicy(sizePolicy3);
        erodeIterationsEdit->setMinimumSize(QSize(50, 27));
        erodeIterationsEdit->setMaximumSize(QSize(50, 16777215));
        erodeIterationsEdit->setFont(font1);

        horizontalLayout_8->addWidget(erodeIterationsEdit);

        label_13 = new QLabel(layoutWidget4);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setMinimumSize(QSize(0, 27));
        label_13->setFont(font);

        horizontalLayout_8->addWidget(label_13);


        verticalLayout_8->addLayout(horizontalLayout_8);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer_4);

        resetErodeToDefaultsButton = new QPushButton(layoutWidget4);
        resetErodeToDefaultsButton->setObjectName(QString::fromUtf8("resetErodeToDefaultsButton"));

        verticalLayout_8->addWidget(resetErodeToDefaultsButton);

        tabWidget->addTab(erodeTab, QString());
        flipTab = new QWidget();
        flipTab->setObjectName(QString::fromUtf8("flipTab"));
        layoutWidget5 = new QWidget(flipTab);
        layoutWidget5->setObjectName(QString::fromUtf8("layoutWidget5"));
        layoutWidget5->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_9 = new QVBoxLayout(layoutWidget5);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        verticalLayout_9->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_12 = new QLabel(layoutWidget5);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        sizePolicy1.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy1);
        label_12->setMinimumSize(QSize(0, 27));
        label_12->setMaximumSize(QSize(16777215, 27));
        label_12->setFont(font);

        horizontalLayout_9->addWidget(label_12);

        flipXAxisButton = new QRadioButton(layoutWidget5);
        flipCodeGroup = new QButtonGroup(ImageProcessingSettingsDialog);
        flipCodeGroup->setObjectName(QString::fromUtf8("flipCodeGroup"));
        flipCodeGroup->addButton(flipXAxisButton);
        flipXAxisButton->setObjectName(QString::fromUtf8("flipXAxisButton"));
        flipXAxisButton->setFont(font1);

        horizontalLayout_9->addWidget(flipXAxisButton);

        flipYAxisButton = new QRadioButton(layoutWidget5);
        flipCodeGroup->addButton(flipYAxisButton);
        flipYAxisButton->setObjectName(QString::fromUtf8("flipYAxisButton"));
        flipYAxisButton->setFont(font1);

        horizontalLayout_9->addWidget(flipYAxisButton);

        flipBothAxesButton = new QRadioButton(layoutWidget5);
        flipCodeGroup->addButton(flipBothAxesButton);
        flipBothAxesButton->setObjectName(QString::fromUtf8("flipBothAxesButton"));
        flipBothAxesButton->setFont(font1);

        horizontalLayout_9->addWidget(flipBothAxesButton);


        verticalLayout_9->addLayout(horizontalLayout_9);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_5);

        resetFlipToDefaultsButton = new QPushButton(layoutWidget5);
        resetFlipToDefaultsButton->setObjectName(QString::fromUtf8("resetFlipToDefaultsButton"));

        verticalLayout_9->addWidget(resetFlipToDefaultsButton);

        tabWidget->addTab(flipTab, QString());
        cannyTab = new QWidget();
        cannyTab->setObjectName(QString::fromUtf8("cannyTab"));
        layoutWidget6 = new QWidget(cannyTab);
        layoutWidget6->setObjectName(QString::fromUtf8("layoutWidget6"));
        layoutWidget6->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_6 = new QVBoxLayout(layoutWidget6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_1 = new QLabel(layoutWidget6);
        label_1->setObjectName(QString::fromUtf8("label_1"));
        label_1->setMinimumSize(QSize(0, 27));
        label_1->setFont(font);

        horizontalLayout_10->addWidget(label_1);

        cannyThresh1Edit = new QLineEdit(layoutWidget6);
        cannyThresh1Edit->setObjectName(QString::fromUtf8("cannyThresh1Edit"));
        sizePolicy3.setHeightForWidth(cannyThresh1Edit->sizePolicy().hasHeightForWidth());
        cannyThresh1Edit->setSizePolicy(sizePolicy3);
        cannyThresh1Edit->setMinimumSize(QSize(50, 27));
        cannyThresh1Edit->setMaximumSize(QSize(50, 16777215));
        cannyThresh1Edit->setFont(font1);

        horizontalLayout_10->addWidget(cannyThresh1Edit);

        label_8 = new QLabel(layoutWidget6);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setMinimumSize(QSize(0, 27));
        label_8->setFont(font);

        horizontalLayout_10->addWidget(label_8);


        verticalLayout_6->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_16 = new QLabel(layoutWidget6);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setMinimumSize(QSize(0, 27));
        label_16->setFont(font);

        horizontalLayout_11->addWidget(label_16);

        cannyThresh2Edit = new QLineEdit(layoutWidget6);
        cannyThresh2Edit->setObjectName(QString::fromUtf8("cannyThresh2Edit"));
        sizePolicy3.setHeightForWidth(cannyThresh2Edit->sizePolicy().hasHeightForWidth());
        cannyThresh2Edit->setSizePolicy(sizePolicy3);
        cannyThresh2Edit->setMinimumSize(QSize(50, 27));
        cannyThresh2Edit->setMaximumSize(QSize(50, 16777215));
        cannyThresh2Edit->setFont(font1);

        horizontalLayout_11->addWidget(cannyThresh2Edit);

        label_20 = new QLabel(layoutWidget6);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setMinimumSize(QSize(0, 27));
        label_20->setFont(font);

        horizontalLayout_11->addWidget(label_20);


        verticalLayout_6->addLayout(horizontalLayout_11);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_10 = new QLabel(layoutWidget6);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setMinimumSize(QSize(0, 27));
        label_10->setFont(font);

        horizontalLayout_12->addWidget(label_10);

        cannyApertureSizeEdit = new QLineEdit(layoutWidget6);
        cannyApertureSizeEdit->setObjectName(QString::fromUtf8("cannyApertureSizeEdit"));
        sizePolicy3.setHeightForWidth(cannyApertureSizeEdit->sizePolicy().hasHeightForWidth());
        cannyApertureSizeEdit->setSizePolicy(sizePolicy3);
        cannyApertureSizeEdit->setMinimumSize(QSize(50, 27));
        cannyApertureSizeEdit->setMaximumSize(QSize(50, 16777215));
        cannyApertureSizeEdit->setFont(font1);

        horizontalLayout_12->addWidget(cannyApertureSizeEdit);

        label_9 = new QLabel(layoutWidget6);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setMinimumSize(QSize(0, 27));
        label_9->setFont(font);

        horizontalLayout_12->addWidget(label_9);


        verticalLayout_6->addLayout(horizontalLayout_12);

        cannyL2NormCheckBox = new QCheckBox(layoutWidget6);
        cannyL2NormCheckBox->setObjectName(QString::fromUtf8("cannyL2NormCheckBox"));
        cannyL2NormCheckBox->setMinimumSize(QSize(0, 27));
        cannyL2NormCheckBox->setFont(font);

        verticalLayout_6->addWidget(cannyL2NormCheckBox);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_6);

        resetCannyToDefaultsButton = new QPushButton(layoutWidget6);
        resetCannyToDefaultsButton->setObjectName(QString::fromUtf8("resetCannyToDefaultsButton"));

        verticalLayout_6->addWidget(resetCannyToDefaultsButton);

        tabWidget->addTab(cannyTab, QString());

        verticalLayout_1->addWidget(tabWidget);

        line_2 = new QFrame(layoutWidget1);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_1->addWidget(line_2);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        applyButton = new QPushButton(layoutWidget1);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        horizontalLayout_6->addWidget(applyButton);

        resetAllToDefaultsButton = new QPushButton(layoutWidget1);
        resetAllToDefaultsButton->setObjectName(QString::fromUtf8("resetAllToDefaultsButton"));

        horizontalLayout_6->addWidget(resetAllToDefaultsButton);


        verticalLayout_1->addLayout(horizontalLayout_6);

        line_1 = new QFrame(layoutWidget1);
        line_1->setObjectName(QString::fromUtf8("line_1"));
        line_1->setFrameShape(QFrame::HLine);
        line_1->setFrameShadow(QFrame::Sunken);

        verticalLayout_1->addWidget(line_1);

        okCancelBox = new QDialogButtonBox(layoutWidget1);
        okCancelBox->setObjectName(QString::fromUtf8("okCancelBox"));
        okCancelBox->setOrientation(Qt::Horizontal);
        okCancelBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout_1->addWidget(okCancelBox);

        QWidget::setTabOrder(tabWidget, smoothBlurButton);
        QWidget::setTabOrder(smoothBlurButton, smoothGaussianButton);
        QWidget::setTabOrder(smoothGaussianButton, smoothMedianButton);
        QWidget::setTabOrder(smoothMedianButton, smoothParam1Edit);
        QWidget::setTabOrder(smoothParam1Edit, smoothParam2Edit);
        QWidget::setTabOrder(smoothParam2Edit, smoothParam3Edit);
        QWidget::setTabOrder(smoothParam3Edit, smoothParam4Edit);
        QWidget::setTabOrder(smoothParam4Edit, resetSmoothToDefaultsButton);
        QWidget::setTabOrder(resetSmoothToDefaultsButton, dilateIterationsEdit);
        QWidget::setTabOrder(dilateIterationsEdit, resetDilateToDefaultsButton);
        QWidget::setTabOrder(resetDilateToDefaultsButton, erodeIterationsEdit);
        QWidget::setTabOrder(erodeIterationsEdit, resetErodeToDefaultsButton);
        QWidget::setTabOrder(resetErodeToDefaultsButton, flipXAxisButton);
        QWidget::setTabOrder(flipXAxisButton, flipYAxisButton);
        QWidget::setTabOrder(flipYAxisButton, flipBothAxesButton);
        QWidget::setTabOrder(flipBothAxesButton, resetFlipToDefaultsButton);
        QWidget::setTabOrder(resetFlipToDefaultsButton, cannyThresh1Edit);
        QWidget::setTabOrder(cannyThresh1Edit, cannyThresh2Edit);
        QWidget::setTabOrder(cannyThresh2Edit, cannyApertureSizeEdit);
        QWidget::setTabOrder(cannyApertureSizeEdit, cannyL2NormCheckBox);
        QWidget::setTabOrder(cannyL2NormCheckBox, resetCannyToDefaultsButton);
        QWidget::setTabOrder(resetCannyToDefaultsButton, applyButton);
        QWidget::setTabOrder(applyButton, resetAllToDefaultsButton);
        QWidget::setTabOrder(resetAllToDefaultsButton, okCancelBox);

        retranslateUi(ImageProcessingSettingsDialog);
        QObject::connect(okCancelBox, SIGNAL(accepted()), ImageProcessingSettingsDialog, SLOT(accept()));
        QObject::connect(okCancelBox, SIGNAL(rejected()), ImageProcessingSettingsDialog, SLOT(reject()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(ImageProcessingSettingsDialog);
    } // setupUi

    void retranslateUi(QDialog *ImageProcessingSettingsDialog)
    {
        ImageProcessingSettingsDialog->setWindowTitle(QApplication::translate("ImageProcessingSettingsDialog", "Image Processing Settings", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("ImageProcessingSettingsDialog", "Type:", 0, QApplication::UnicodeUTF8));
        smoothBlurButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Blur", 0, QApplication::UnicodeUTF8));
        smoothGaussianButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Gaussian", 0, QApplication::UnicodeUTF8));
        smoothMedianButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Median", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("ImageProcessingSettingsDialog", "Parameters:", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("ImageProcessingSettingsDialog", "1:", 0, QApplication::UnicodeUTF8));
        smoothParam1RangeLabel->setText(QString());
        smoothParam1Label->setText(QString());
        label_5->setText(QApplication::translate("ImageProcessingSettingsDialog", "2:", 0, QApplication::UnicodeUTF8));
        smoothParam2RangeLabel->setText(QString());
        smoothParam2Label->setText(QString());
        label_6->setText(QApplication::translate("ImageProcessingSettingsDialog", "3:", 0, QApplication::UnicodeUTF8));
        smoothParam3RangeLabel->setText(QString());
        smoothParam3Label->setText(QString());
        label_7->setText(QApplication::translate("ImageProcessingSettingsDialog", "4:", 0, QApplication::UnicodeUTF8));
        smoothParam4RangeLabel->setText(QString());
        smoothParam4Label->setText(QString());
        resetSmoothToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(smoothTab), QApplication::translate("ImageProcessingSettingsDialog", "Smooth", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("ImageProcessingSettingsDialog", "Number of iterations:", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ImageProcessingSettingsDialog", "[1-99]", 0, QApplication::UnicodeUTF8));
        resetDilateToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(dilateTab), QApplication::translate("ImageProcessingSettingsDialog", "Dilate", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("ImageProcessingSettingsDialog", "Number of iterations:", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("ImageProcessingSettingsDialog", "[1-99]", 0, QApplication::UnicodeUTF8));
        resetErodeToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(erodeTab), QApplication::translate("ImageProcessingSettingsDialog", "Erode", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("ImageProcessingSettingsDialog", "Mode:", 0, QApplication::UnicodeUTF8));
        flipXAxisButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "X-axis", 0, QApplication::UnicodeUTF8));
        flipYAxisButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Y-axis", 0, QApplication::UnicodeUTF8));
        flipBothAxesButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Both axes", 0, QApplication::UnicodeUTF8));
        resetFlipToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(flipTab), QApplication::translate("ImageProcessingSettingsDialog", "Flip", 0, QApplication::UnicodeUTF8));
        label_1->setText(QApplication::translate("ImageProcessingSettingsDialog", "Threshold 1:", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("ImageProcessingSettingsDialog", "[0-999]", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("ImageProcessingSettingsDialog", "Threshold 2:", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("ImageProcessingSettingsDialog", "[0-999]", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("ImageProcessingSettingsDialog", "Aperture Size:", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("ImageProcessingSettingsDialog", "[3/5/7]", 0, QApplication::UnicodeUTF8));
        cannyL2NormCheckBox->setText(QApplication::translate("ImageProcessingSettingsDialog", "Use L2-norm", 0, QApplication::UnicodeUTF8));
        resetCannyToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(cannyTab), QApplication::translate("ImageProcessingSettingsDialog", "Canny", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Apply", 0, QApplication::UnicodeUTF8));
        resetAllToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset All to Defaults", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ImageProcessingSettingsDialog: public Ui_ImageProcessingSettingsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGEPROCESSINGSETTINGSDIALOG_H
