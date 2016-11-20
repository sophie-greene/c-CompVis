/********************************************************************************
** Form generated from reading UI file 'ImageProcessingSettingsDialog.ui'
**
** Created: Thu May 23 11:55:13 2013
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
    QVBoxLayout *verticalLayout_40;
    QTabWidget *tabWidget;
    QWidget *smoothTab_5;
    QWidget *layoutWidget2_5;
    QVBoxLayout *verticalLayout_41;
    QHBoxLayout *horizontalLayout_48;
    QVBoxLayout *verticalLayout_42;
    QLabel *label_68;
    QVBoxLayout *verticalLayout_43;
    QRadioButton *smoothBlurButton;
    QRadioButton *smoothGaussianButton;
    QRadioButton *smoothMedianButton;
    QSpacerItem *verticalSpacer_29;
    QVBoxLayout *verticalLayout_44;
    QLabel *label_69;
    QVBoxLayout *verticalLayout_45;
    QHBoxLayout *horizontalLayout_49;
    QLabel *label_70;
    QLineEdit *smoothParam1Edit;
    QLabel *smoothParam1RangeLabel;
    QLabel *smoothParam1Label;
    QHBoxLayout *horizontalLayout_50;
    QLabel *label_71;
    QLineEdit *smoothParam2Edit;
    QLabel *smoothParam2RangeLabel;
    QLabel *smoothParam2Label;
    QHBoxLayout *horizontalLayout_51;
    QLabel *label_72;
    QLineEdit *smoothParam3Edit;
    QLabel *smoothParam3RangeLabel;
    QLabel *smoothParam3Label;
    QHBoxLayout *horizontalLayout_52;
    QLabel *label_73;
    QLineEdit *smoothParam4Edit;
    QLabel *smoothParam4RangeLabel;
    QLabel *smoothParam4Label;
    QSpacerItem *verticalSpacer_31;
    QPushButton *resetSmoothToDefaultsButton;
    QWidget *dilateTab_5;
    QWidget *layoutWidget3_5;
    QVBoxLayout *verticalLayout_46;
    QHBoxLayout *horizontalLayout_53;
    QLabel *label_74;
    QLineEdit *dilateIterationsEdit;
    QLabel *label_75;
    QSpacerItem *verticalSpacer_32;
    QPushButton *resetDilateToDefaultsButton;
    QWidget *erodeTab_5;
    QWidget *layoutWidget4_5;
    QVBoxLayout *verticalLayout_47;
    QHBoxLayout *horizontalLayout_54;
    QLabel *label_76;
    QLineEdit *erodeIterationsEdit;
    QLabel *label_77;
    QSpacerItem *verticalSpacer_33;
    QPushButton *resetErodeToDefaultsButton;
    QWidget *flipTab_5;
    QWidget *layoutWidget5_5;
    QVBoxLayout *verticalLayout_48;
    QHBoxLayout *horizontalLayout_55;
    QLabel *label_78;
    QRadioButton *flipXAxisButton;
    QRadioButton *flipYAxisButton;
    QRadioButton *flipBothAxesButton;
    QSpacerItem *verticalSpacer_34;
    QPushButton *resetFlipToDefaultsButton;
    QWidget *cannyTab_5;
    QWidget *layoutWidget6_5;
    QVBoxLayout *verticalLayout_49;
    QHBoxLayout *horizontalLayout_56;
    QLabel *label_79;
    QLineEdit *cannyThresh1Edit;
    QLabel *label_80;
    QHBoxLayout *horizontalLayout_57;
    QLabel *label_81;
    QLineEdit *cannyThresh2Edit;
    QLabel *label_82;
    QHBoxLayout *horizontalLayout_58;
    QLabel *label_83;
    QLineEdit *cannyApertureSizeEdit;
    QLabel *label_84;
    QCheckBox *cannyL2NormCheckBox;
    QSpacerItem *verticalSpacer_35;
    QPushButton *resetCannyToDefaultsButton;
    QFrame *line_9;
    QHBoxLayout *horizontalLayout_59;
    QPushButton *applyButton;
    QPushButton *resetAllToDefaultsButton;
    QFrame *line_10;
    QDialogButtonBox *okCancelBox;
    QButtonGroup *flipCodeGroup;
    QButtonGroup *smoothTypeGroup;

    void setupUi(QDialog *ImageProcessingSettingsDialog)
    {
        if (ImageProcessingSettingsDialog->objectName().isEmpty())
            ImageProcessingSettingsDialog->setObjectName(QString::fromUtf8("ImageProcessingSettingsDialog"));
        ImageProcessingSettingsDialog->resize(440, 380);
        layoutWidget1 = new QWidget(ImageProcessingSettingsDialog);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 10, 421, 361));
        verticalLayout_40 = new QVBoxLayout(layoutWidget1);
        verticalLayout_40->setObjectName(QString::fromUtf8("verticalLayout_40"));
        verticalLayout_40->setContentsMargins(0, 0, 0, 0);
        tabWidget = new QTabWidget(layoutWidget1);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        smoothTab_5 = new QWidget();
        smoothTab_5->setObjectName(QString::fromUtf8("smoothTab_5"));
        layoutWidget2_5 = new QWidget(smoothTab_5);
        layoutWidget2_5->setObjectName(QString::fromUtf8("layoutWidget2_5"));
        layoutWidget2_5->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_41 = new QVBoxLayout(layoutWidget2_5);
        verticalLayout_41->setObjectName(QString::fromUtf8("verticalLayout_41"));
        verticalLayout_41->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_48 = new QHBoxLayout();
        horizontalLayout_48->setObjectName(QString::fromUtf8("horizontalLayout_48"));
        verticalLayout_42 = new QVBoxLayout();
        verticalLayout_42->setObjectName(QString::fromUtf8("verticalLayout_42"));
        label_68 = new QLabel(layoutWidget2_5);
        label_68->setObjectName(QString::fromUtf8("label_68"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_68->sizePolicy().hasHeightForWidth());
        label_68->setSizePolicy(sizePolicy);
        label_68->setMinimumSize(QSize(0, 27));
        label_68->setMaximumSize(QSize(140, 27));
        QFont font;
        font.setPointSize(8);
        font.setBold(true);
        font.setWeight(75);
        label_68->setFont(font);

        verticalLayout_42->addWidget(label_68);

        verticalLayout_43 = new QVBoxLayout();
        verticalLayout_43->setObjectName(QString::fromUtf8("verticalLayout_43"));
        smoothBlurButton = new QRadioButton(layoutWidget2_5);
        smoothTypeGroup = new QButtonGroup(ImageProcessingSettingsDialog);
        smoothTypeGroup->setObjectName(QString::fromUtf8("smoothTypeGroup"));
        smoothTypeGroup->addButton(smoothBlurButton);
        smoothBlurButton->setObjectName(QString::fromUtf8("smoothBlurButton"));
        smoothBlurButton->setMaximumSize(QSize(140, 16777215));
        QFont font1;
        font1.setPointSize(8);
        smoothBlurButton->setFont(font1);

        verticalLayout_43->addWidget(smoothBlurButton);

        smoothGaussianButton = new QRadioButton(layoutWidget2_5);
        smoothTypeGroup->addButton(smoothGaussianButton);
        smoothGaussianButton->setObjectName(QString::fromUtf8("smoothGaussianButton"));
        smoothGaussianButton->setMaximumSize(QSize(140, 16777215));
        smoothGaussianButton->setFont(font1);

        verticalLayout_43->addWidget(smoothGaussianButton);

        smoothMedianButton = new QRadioButton(layoutWidget2_5);
        smoothTypeGroup->addButton(smoothMedianButton);
        smoothMedianButton->setObjectName(QString::fromUtf8("smoothMedianButton"));
        smoothMedianButton->setMaximumSize(QSize(140, 16777215));
        smoothMedianButton->setFont(font1);

        verticalLayout_43->addWidget(smoothMedianButton);

        verticalSpacer_29 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_43->addItem(verticalSpacer_29);


        verticalLayout_42->addLayout(verticalLayout_43);


        horizontalLayout_48->addLayout(verticalLayout_42);

        verticalLayout_44 = new QVBoxLayout();
        verticalLayout_44->setObjectName(QString::fromUtf8("verticalLayout_44"));
        label_69 = new QLabel(layoutWidget2_5);
        label_69->setObjectName(QString::fromUtf8("label_69"));
        label_69->setMinimumSize(QSize(0, 27));
        label_69->setMaximumSize(QSize(16777215, 27));
        label_69->setFont(font);

        verticalLayout_44->addWidget(label_69);

        verticalLayout_45 = new QVBoxLayout();
        verticalLayout_45->setObjectName(QString::fromUtf8("verticalLayout_45"));
        horizontalLayout_49 = new QHBoxLayout();
        horizontalLayout_49->setObjectName(QString::fromUtf8("horizontalLayout_49"));
        label_70 = new QLabel(layoutWidget2_5);
        label_70->setObjectName(QString::fromUtf8("label_70"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_70->sizePolicy().hasHeightForWidth());
        label_70->setSizePolicy(sizePolicy1);
        label_70->setMinimumSize(QSize(15, 0));
        label_70->setMaximumSize(QSize(15, 16777215));
        label_70->setFont(font1);

        horizontalLayout_49->addWidget(label_70);

        smoothParam1Edit = new QLineEdit(layoutWidget2_5);
        smoothParam1Edit->setObjectName(QString::fromUtf8("smoothParam1Edit"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(smoothParam1Edit->sizePolicy().hasHeightForWidth());
        smoothParam1Edit->setSizePolicy(sizePolicy2);
        smoothParam1Edit->setMinimumSize(QSize(45, 0));
        smoothParam1Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam1Edit->setFont(font1);

        horizontalLayout_49->addWidget(smoothParam1Edit);

        smoothParam1RangeLabel = new QLabel(layoutWidget2_5);
        smoothParam1RangeLabel->setObjectName(QString::fromUtf8("smoothParam1RangeLabel"));
        smoothParam1RangeLabel->setFont(font);

        horizontalLayout_49->addWidget(smoothParam1RangeLabel);

        smoothParam1Label = new QLabel(layoutWidget2_5);
        smoothParam1Label->setObjectName(QString::fromUtf8("smoothParam1Label"));
        smoothParam1Label->setFont(font1);

        horizontalLayout_49->addWidget(smoothParam1Label);


        verticalLayout_45->addLayout(horizontalLayout_49);

        horizontalLayout_50 = new QHBoxLayout();
        horizontalLayout_50->setObjectName(QString::fromUtf8("horizontalLayout_50"));
        label_71 = new QLabel(layoutWidget2_5);
        label_71->setObjectName(QString::fromUtf8("label_71"));
        sizePolicy1.setHeightForWidth(label_71->sizePolicy().hasHeightForWidth());
        label_71->setSizePolicy(sizePolicy1);
        label_71->setMinimumSize(QSize(15, 0));
        label_71->setMaximumSize(QSize(15, 16777215));
        label_71->setFont(font1);

        horizontalLayout_50->addWidget(label_71);

        smoothParam2Edit = new QLineEdit(layoutWidget2_5);
        smoothParam2Edit->setObjectName(QString::fromUtf8("smoothParam2Edit"));
        sizePolicy2.setHeightForWidth(smoothParam2Edit->sizePolicy().hasHeightForWidth());
        smoothParam2Edit->setSizePolicy(sizePolicy2);
        smoothParam2Edit->setMinimumSize(QSize(45, 0));
        smoothParam2Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam2Edit->setFont(font1);

        horizontalLayout_50->addWidget(smoothParam2Edit);

        smoothParam2RangeLabel = new QLabel(layoutWidget2_5);
        smoothParam2RangeLabel->setObjectName(QString::fromUtf8("smoothParam2RangeLabel"));
        smoothParam2RangeLabel->setFont(font);

        horizontalLayout_50->addWidget(smoothParam2RangeLabel);

        smoothParam2Label = new QLabel(layoutWidget2_5);
        smoothParam2Label->setObjectName(QString::fromUtf8("smoothParam2Label"));
        smoothParam2Label->setFont(font1);

        horizontalLayout_50->addWidget(smoothParam2Label);


        verticalLayout_45->addLayout(horizontalLayout_50);

        horizontalLayout_51 = new QHBoxLayout();
        horizontalLayout_51->setObjectName(QString::fromUtf8("horizontalLayout_51"));
        label_72 = new QLabel(layoutWidget2_5);
        label_72->setObjectName(QString::fromUtf8("label_72"));
        sizePolicy1.setHeightForWidth(label_72->sizePolicy().hasHeightForWidth());
        label_72->setSizePolicy(sizePolicy1);
        label_72->setMinimumSize(QSize(15, 0));
        label_72->setMaximumSize(QSize(15, 16777215));
        label_72->setFont(font1);

        horizontalLayout_51->addWidget(label_72);

        smoothParam3Edit = new QLineEdit(layoutWidget2_5);
        smoothParam3Edit->setObjectName(QString::fromUtf8("smoothParam3Edit"));
        sizePolicy2.setHeightForWidth(smoothParam3Edit->sizePolicy().hasHeightForWidth());
        smoothParam3Edit->setSizePolicy(sizePolicy2);
        smoothParam3Edit->setMinimumSize(QSize(45, 0));
        smoothParam3Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam3Edit->setFont(font1);

        horizontalLayout_51->addWidget(smoothParam3Edit);

        smoothParam3RangeLabel = new QLabel(layoutWidget2_5);
        smoothParam3RangeLabel->setObjectName(QString::fromUtf8("smoothParam3RangeLabel"));
        smoothParam3RangeLabel->setFont(font);

        horizontalLayout_51->addWidget(smoothParam3RangeLabel);

        smoothParam3Label = new QLabel(layoutWidget2_5);
        smoothParam3Label->setObjectName(QString::fromUtf8("smoothParam3Label"));
        smoothParam3Label->setFont(font1);

        horizontalLayout_51->addWidget(smoothParam3Label);


        verticalLayout_45->addLayout(horizontalLayout_51);

        horizontalLayout_52 = new QHBoxLayout();
        horizontalLayout_52->setObjectName(QString::fromUtf8("horizontalLayout_52"));
        label_73 = new QLabel(layoutWidget2_5);
        label_73->setObjectName(QString::fromUtf8("label_73"));
        sizePolicy1.setHeightForWidth(label_73->sizePolicy().hasHeightForWidth());
        label_73->setSizePolicy(sizePolicy1);
        label_73->setMinimumSize(QSize(15, 0));
        label_73->setMaximumSize(QSize(15, 16777215));
        label_73->setFont(font1);

        horizontalLayout_52->addWidget(label_73);

        smoothParam4Edit = new QLineEdit(layoutWidget2_5);
        smoothParam4Edit->setObjectName(QString::fromUtf8("smoothParam4Edit"));
        sizePolicy2.setHeightForWidth(smoothParam4Edit->sizePolicy().hasHeightForWidth());
        smoothParam4Edit->setSizePolicy(sizePolicy2);
        smoothParam4Edit->setMinimumSize(QSize(45, 0));
        smoothParam4Edit->setMaximumSize(QSize(45, 16777215));
        smoothParam4Edit->setFont(font1);

        horizontalLayout_52->addWidget(smoothParam4Edit);

        smoothParam4RangeLabel = new QLabel(layoutWidget2_5);
        smoothParam4RangeLabel->setObjectName(QString::fromUtf8("smoothParam4RangeLabel"));
        smoothParam4RangeLabel->setFont(font);

        horizontalLayout_52->addWidget(smoothParam4RangeLabel);

        smoothParam4Label = new QLabel(layoutWidget2_5);
        smoothParam4Label->setObjectName(QString::fromUtf8("smoothParam4Label"));
        smoothParam4Label->setFont(font1);

        horizontalLayout_52->addWidget(smoothParam4Label);


        verticalLayout_45->addLayout(horizontalLayout_52);


        verticalLayout_44->addLayout(verticalLayout_45);


        horizontalLayout_48->addLayout(verticalLayout_44);


        verticalLayout_41->addLayout(horizontalLayout_48);

        verticalSpacer_31 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_41->addItem(verticalSpacer_31);

        resetSmoothToDefaultsButton = new QPushButton(layoutWidget2_5);
        resetSmoothToDefaultsButton->setObjectName(QString::fromUtf8("resetSmoothToDefaultsButton"));

        verticalLayout_41->addWidget(resetSmoothToDefaultsButton);

        tabWidget->addTab(smoothTab_5, QString());
        dilateTab_5 = new QWidget();
        dilateTab_5->setObjectName(QString::fromUtf8("dilateTab_5"));
        layoutWidget3_5 = new QWidget(dilateTab_5);
        layoutWidget3_5->setObjectName(QString::fromUtf8("layoutWidget3_5"));
        layoutWidget3_5->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_46 = new QVBoxLayout(layoutWidget3_5);
        verticalLayout_46->setObjectName(QString::fromUtf8("verticalLayout_46"));
        verticalLayout_46->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_53 = new QHBoxLayout();
        horizontalLayout_53->setObjectName(QString::fromUtf8("horizontalLayout_53"));
        label_74 = new QLabel(layoutWidget3_5);
        label_74->setObjectName(QString::fromUtf8("label_74"));
        label_74->setMinimumSize(QSize(0, 27));
        label_74->setFont(font);

        horizontalLayout_53->addWidget(label_74);

        dilateIterationsEdit = new QLineEdit(layoutWidget3_5);
        dilateIterationsEdit->setObjectName(QString::fromUtf8("dilateIterationsEdit"));
        sizePolicy2.setHeightForWidth(dilateIterationsEdit->sizePolicy().hasHeightForWidth());
        dilateIterationsEdit->setSizePolicy(sizePolicy2);
        dilateIterationsEdit->setMinimumSize(QSize(50, 27));
        dilateIterationsEdit->setMaximumSize(QSize(50, 16777215));
        dilateIterationsEdit->setFont(font1);

        horizontalLayout_53->addWidget(dilateIterationsEdit);

        label_75 = new QLabel(layoutWidget3_5);
        label_75->setObjectName(QString::fromUtf8("label_75"));
        label_75->setMinimumSize(QSize(0, 27));
        label_75->setFont(font);

        horizontalLayout_53->addWidget(label_75);


        verticalLayout_46->addLayout(horizontalLayout_53);

        verticalSpacer_32 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_46->addItem(verticalSpacer_32);

        resetDilateToDefaultsButton = new QPushButton(layoutWidget3_5);
        resetDilateToDefaultsButton->setObjectName(QString::fromUtf8("resetDilateToDefaultsButton"));

        verticalLayout_46->addWidget(resetDilateToDefaultsButton);

        tabWidget->addTab(dilateTab_5, QString());
        erodeTab_5 = new QWidget();
        erodeTab_5->setObjectName(QString::fromUtf8("erodeTab_5"));
        layoutWidget4_5 = new QWidget(erodeTab_5);
        layoutWidget4_5->setObjectName(QString::fromUtf8("layoutWidget4_5"));
        layoutWidget4_5->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_47 = new QVBoxLayout(layoutWidget4_5);
        verticalLayout_47->setObjectName(QString::fromUtf8("verticalLayout_47"));
        verticalLayout_47->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_54 = new QHBoxLayout();
        horizontalLayout_54->setObjectName(QString::fromUtf8("horizontalLayout_54"));
        label_76 = new QLabel(layoutWidget4_5);
        label_76->setObjectName(QString::fromUtf8("label_76"));
        label_76->setMinimumSize(QSize(0, 27));
        label_76->setFont(font);

        horizontalLayout_54->addWidget(label_76);

        erodeIterationsEdit = new QLineEdit(layoutWidget4_5);
        erodeIterationsEdit->setObjectName(QString::fromUtf8("erodeIterationsEdit"));
        sizePolicy2.setHeightForWidth(erodeIterationsEdit->sizePolicy().hasHeightForWidth());
        erodeIterationsEdit->setSizePolicy(sizePolicy2);
        erodeIterationsEdit->setMinimumSize(QSize(50, 27));
        erodeIterationsEdit->setMaximumSize(QSize(50, 16777215));
        erodeIterationsEdit->setFont(font1);

        horizontalLayout_54->addWidget(erodeIterationsEdit);

        label_77 = new QLabel(layoutWidget4_5);
        label_77->setObjectName(QString::fromUtf8("label_77"));
        label_77->setMinimumSize(QSize(0, 27));
        label_77->setFont(font);

        horizontalLayout_54->addWidget(label_77);


        verticalLayout_47->addLayout(horizontalLayout_54);

        verticalSpacer_33 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_47->addItem(verticalSpacer_33);

        resetErodeToDefaultsButton = new QPushButton(layoutWidget4_5);
        resetErodeToDefaultsButton->setObjectName(QString::fromUtf8("resetErodeToDefaultsButton"));

        verticalLayout_47->addWidget(resetErodeToDefaultsButton);

        tabWidget->addTab(erodeTab_5, QString());
        flipTab_5 = new QWidget();
        flipTab_5->setObjectName(QString::fromUtf8("flipTab_5"));
        layoutWidget5_5 = new QWidget(flipTab_5);
        layoutWidget5_5->setObjectName(QString::fromUtf8("layoutWidget5_5"));
        layoutWidget5_5->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_48 = new QVBoxLayout(layoutWidget5_5);
        verticalLayout_48->setObjectName(QString::fromUtf8("verticalLayout_48"));
        verticalLayout_48->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_55 = new QHBoxLayout();
        horizontalLayout_55->setObjectName(QString::fromUtf8("horizontalLayout_55"));
        label_78 = new QLabel(layoutWidget5_5);
        label_78->setObjectName(QString::fromUtf8("label_78"));
        sizePolicy.setHeightForWidth(label_78->sizePolicy().hasHeightForWidth());
        label_78->setSizePolicy(sizePolicy);
        label_78->setMinimumSize(QSize(0, 27));
        label_78->setMaximumSize(QSize(16777215, 27));
        label_78->setFont(font);

        horizontalLayout_55->addWidget(label_78);

        flipXAxisButton = new QRadioButton(layoutWidget5_5);
        flipCodeGroup = new QButtonGroup(ImageProcessingSettingsDialog);
        flipCodeGroup->setObjectName(QString::fromUtf8("flipCodeGroup"));
        flipCodeGroup->addButton(flipXAxisButton);
        flipXAxisButton->setObjectName(QString::fromUtf8("flipXAxisButton"));
        flipXAxisButton->setFont(font1);

        horizontalLayout_55->addWidget(flipXAxisButton);

        flipYAxisButton = new QRadioButton(layoutWidget5_5);
        flipCodeGroup->addButton(flipYAxisButton);
        flipYAxisButton->setObjectName(QString::fromUtf8("flipYAxisButton"));
        flipYAxisButton->setFont(font1);

        horizontalLayout_55->addWidget(flipYAxisButton);

        flipBothAxesButton = new QRadioButton(layoutWidget5_5);
        flipCodeGroup->addButton(flipBothAxesButton);
        flipBothAxesButton->setObjectName(QString::fromUtf8("flipBothAxesButton"));
        flipBothAxesButton->setFont(font1);

        horizontalLayout_55->addWidget(flipBothAxesButton);


        verticalLayout_48->addLayout(horizontalLayout_55);

        verticalSpacer_34 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_48->addItem(verticalSpacer_34);

        resetFlipToDefaultsButton = new QPushButton(layoutWidget5_5);
        resetFlipToDefaultsButton->setObjectName(QString::fromUtf8("resetFlipToDefaultsButton"));

        verticalLayout_48->addWidget(resetFlipToDefaultsButton);

        tabWidget->addTab(flipTab_5, QString());
        cannyTab_5 = new QWidget();
        cannyTab_5->setObjectName(QString::fromUtf8("cannyTab_5"));
        layoutWidget6_5 = new QWidget(cannyTab_5);
        layoutWidget6_5->setObjectName(QString::fromUtf8("layoutWidget6_5"));
        layoutWidget6_5->setGeometry(QRect(10, 10, 401, 221));
        verticalLayout_49 = new QVBoxLayout(layoutWidget6_5);
        verticalLayout_49->setObjectName(QString::fromUtf8("verticalLayout_49"));
        verticalLayout_49->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_56 = new QHBoxLayout();
        horizontalLayout_56->setObjectName(QString::fromUtf8("horizontalLayout_56"));
        label_79 = new QLabel(layoutWidget6_5);
        label_79->setObjectName(QString::fromUtf8("label_79"));
        label_79->setMinimumSize(QSize(0, 27));
        label_79->setFont(font);

        horizontalLayout_56->addWidget(label_79);

        cannyThresh1Edit = new QLineEdit(layoutWidget6_5);
        cannyThresh1Edit->setObjectName(QString::fromUtf8("cannyThresh1Edit"));
        sizePolicy2.setHeightForWidth(cannyThresh1Edit->sizePolicy().hasHeightForWidth());
        cannyThresh1Edit->setSizePolicy(sizePolicy2);
        cannyThresh1Edit->setMinimumSize(QSize(50, 27));
        cannyThresh1Edit->setMaximumSize(QSize(50, 16777215));
        cannyThresh1Edit->setFont(font1);

        horizontalLayout_56->addWidget(cannyThresh1Edit);

        label_80 = new QLabel(layoutWidget6_5);
        label_80->setObjectName(QString::fromUtf8("label_80"));
        label_80->setMinimumSize(QSize(0, 27));
        label_80->setFont(font);

        horizontalLayout_56->addWidget(label_80);


        verticalLayout_49->addLayout(horizontalLayout_56);

        horizontalLayout_57 = new QHBoxLayout();
        horizontalLayout_57->setObjectName(QString::fromUtf8("horizontalLayout_57"));
        label_81 = new QLabel(layoutWidget6_5);
        label_81->setObjectName(QString::fromUtf8("label_81"));
        label_81->setMinimumSize(QSize(0, 27));
        label_81->setFont(font);

        horizontalLayout_57->addWidget(label_81);

        cannyThresh2Edit = new QLineEdit(layoutWidget6_5);
        cannyThresh2Edit->setObjectName(QString::fromUtf8("cannyThresh2Edit"));
        sizePolicy2.setHeightForWidth(cannyThresh2Edit->sizePolicy().hasHeightForWidth());
        cannyThresh2Edit->setSizePolicy(sizePolicy2);
        cannyThresh2Edit->setMinimumSize(QSize(50, 27));
        cannyThresh2Edit->setMaximumSize(QSize(50, 16777215));
        cannyThresh2Edit->setFont(font1);

        horizontalLayout_57->addWidget(cannyThresh2Edit);

        label_82 = new QLabel(layoutWidget6_5);
        label_82->setObjectName(QString::fromUtf8("label_82"));
        label_82->setMinimumSize(QSize(0, 27));
        label_82->setFont(font);

        horizontalLayout_57->addWidget(label_82);


        verticalLayout_49->addLayout(horizontalLayout_57);

        horizontalLayout_58 = new QHBoxLayout();
        horizontalLayout_58->setObjectName(QString::fromUtf8("horizontalLayout_58"));
        label_83 = new QLabel(layoutWidget6_5);
        label_83->setObjectName(QString::fromUtf8("label_83"));
        label_83->setMinimumSize(QSize(0, 27));
        label_83->setFont(font);

        horizontalLayout_58->addWidget(label_83);

        cannyApertureSizeEdit = new QLineEdit(layoutWidget6_5);
        cannyApertureSizeEdit->setObjectName(QString::fromUtf8("cannyApertureSizeEdit"));
        sizePolicy2.setHeightForWidth(cannyApertureSizeEdit->sizePolicy().hasHeightForWidth());
        cannyApertureSizeEdit->setSizePolicy(sizePolicy2);
        cannyApertureSizeEdit->setMinimumSize(QSize(50, 27));
        cannyApertureSizeEdit->setMaximumSize(QSize(50, 16777215));
        cannyApertureSizeEdit->setFont(font1);

        horizontalLayout_58->addWidget(cannyApertureSizeEdit);

        label_84 = new QLabel(layoutWidget6_5);
        label_84->setObjectName(QString::fromUtf8("label_84"));
        label_84->setMinimumSize(QSize(0, 27));
        label_84->setFont(font);

        horizontalLayout_58->addWidget(label_84);


        verticalLayout_49->addLayout(horizontalLayout_58);

        cannyL2NormCheckBox = new QCheckBox(layoutWidget6_5);
        cannyL2NormCheckBox->setObjectName(QString::fromUtf8("cannyL2NormCheckBox"));
        cannyL2NormCheckBox->setMinimumSize(QSize(0, 27));
        cannyL2NormCheckBox->setFont(font);

        verticalLayout_49->addWidget(cannyL2NormCheckBox);

        verticalSpacer_35 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_49->addItem(verticalSpacer_35);

        resetCannyToDefaultsButton = new QPushButton(layoutWidget6_5);
        resetCannyToDefaultsButton->setObjectName(QString::fromUtf8("resetCannyToDefaultsButton"));

        verticalLayout_49->addWidget(resetCannyToDefaultsButton);

        tabWidget->addTab(cannyTab_5, QString());

        verticalLayout_40->addWidget(tabWidget);

        line_9 = new QFrame(layoutWidget1);
        line_9->setObjectName(QString::fromUtf8("line_9"));
        line_9->setFrameShape(QFrame::HLine);
        line_9->setFrameShadow(QFrame::Sunken);

        verticalLayout_40->addWidget(line_9);

        horizontalLayout_59 = new QHBoxLayout();
        horizontalLayout_59->setObjectName(QString::fromUtf8("horizontalLayout_59"));
        applyButton = new QPushButton(layoutWidget1);
        applyButton->setObjectName(QString::fromUtf8("applyButton"));

        horizontalLayout_59->addWidget(applyButton);

        resetAllToDefaultsButton = new QPushButton(layoutWidget1);
        resetAllToDefaultsButton->setObjectName(QString::fromUtf8("resetAllToDefaultsButton"));

        horizontalLayout_59->addWidget(resetAllToDefaultsButton);


        verticalLayout_40->addLayout(horizontalLayout_59);

        line_10 = new QFrame(layoutWidget1);
        line_10->setObjectName(QString::fromUtf8("line_10"));
        line_10->setFrameShape(QFrame::HLine);
        line_10->setFrameShadow(QFrame::Sunken);

        verticalLayout_40->addWidget(line_10);

        okCancelBox = new QDialogButtonBox(layoutWidget1);
        okCancelBox->setObjectName(QString::fromUtf8("okCancelBox"));
        okCancelBox->setOrientation(Qt::Horizontal);
        okCancelBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout_40->addWidget(okCancelBox);

        QWidget::setTabOrder(tabWidget, applyButton);
        QWidget::setTabOrder(applyButton, resetAllToDefaultsButton);
        QWidget::setTabOrder(resetAllToDefaultsButton, okCancelBox);
        QWidget::setTabOrder(okCancelBox, smoothBlurButton);
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

        retranslateUi(ImageProcessingSettingsDialog);
        QObject::connect(okCancelBox, SIGNAL(accepted()), ImageProcessingSettingsDialog, SLOT(accept()));
        QObject::connect(okCancelBox, SIGNAL(rejected()), ImageProcessingSettingsDialog, SLOT(reject()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(ImageProcessingSettingsDialog);
    } // setupUi

    void retranslateUi(QDialog *ImageProcessingSettingsDialog)
    {
        ImageProcessingSettingsDialog->setWindowTitle(QApplication::translate("ImageProcessingSettingsDialog", "Image Processing Settings", 0, QApplication::UnicodeUTF8));
        label_68->setText(QApplication::translate("ImageProcessingSettingsDialog", "Type:", 0, QApplication::UnicodeUTF8));
        smoothBlurButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Blur", 0, QApplication::UnicodeUTF8));
        smoothGaussianButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Gaussian", 0, QApplication::UnicodeUTF8));
        smoothMedianButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Median", 0, QApplication::UnicodeUTF8));
        label_69->setText(QApplication::translate("ImageProcessingSettingsDialog", "Parameters:", 0, QApplication::UnicodeUTF8));
        label_70->setText(QApplication::translate("ImageProcessingSettingsDialog", "1:", 0, QApplication::UnicodeUTF8));
        smoothParam1RangeLabel->setText(QString());
        smoothParam1Label->setText(QString());
        label_71->setText(QApplication::translate("ImageProcessingSettingsDialog", "2:", 0, QApplication::UnicodeUTF8));
        smoothParam2RangeLabel->setText(QString());
        smoothParam2Label->setText(QString());
        label_72->setText(QApplication::translate("ImageProcessingSettingsDialog", "3:", 0, QApplication::UnicodeUTF8));
        smoothParam3RangeLabel->setText(QString());
        smoothParam3Label->setText(QString());
        label_73->setText(QApplication::translate("ImageProcessingSettingsDialog", "4:", 0, QApplication::UnicodeUTF8));
        smoothParam4RangeLabel->setText(QString());
        smoothParam4Label->setText(QString());
        resetSmoothToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(smoothTab_5), QApplication::translate("ImageProcessingSettingsDialog", "Smooth", 0, QApplication::UnicodeUTF8));
        label_74->setText(QApplication::translate("ImageProcessingSettingsDialog", "Number of iterations:", 0, QApplication::UnicodeUTF8));
        label_75->setText(QApplication::translate("ImageProcessingSettingsDialog", "[1-99]", 0, QApplication::UnicodeUTF8));
        resetDilateToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(dilateTab_5), QApplication::translate("ImageProcessingSettingsDialog", "Dilate", 0, QApplication::UnicodeUTF8));
        label_76->setText(QApplication::translate("ImageProcessingSettingsDialog", "Number of iterations:", 0, QApplication::UnicodeUTF8));
        label_77->setText(QApplication::translate("ImageProcessingSettingsDialog", "[1-99]", 0, QApplication::UnicodeUTF8));
        resetErodeToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(erodeTab_5), QApplication::translate("ImageProcessingSettingsDialog", "Erode", 0, QApplication::UnicodeUTF8));
        label_78->setText(QApplication::translate("ImageProcessingSettingsDialog", "Mode:", 0, QApplication::UnicodeUTF8));
        flipXAxisButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "X-axis", 0, QApplication::UnicodeUTF8));
        flipYAxisButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Y-axis", 0, QApplication::UnicodeUTF8));
        flipBothAxesButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Both axes", 0, QApplication::UnicodeUTF8));
        resetFlipToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(flipTab_5), QApplication::translate("ImageProcessingSettingsDialog", "Flip", 0, QApplication::UnicodeUTF8));
        label_79->setText(QApplication::translate("ImageProcessingSettingsDialog", "Threshold 1:", 0, QApplication::UnicodeUTF8));
        label_80->setText(QApplication::translate("ImageProcessingSettingsDialog", "[0-999]", 0, QApplication::UnicodeUTF8));
        label_81->setText(QApplication::translate("ImageProcessingSettingsDialog", "Threshold 2:", 0, QApplication::UnicodeUTF8));
        label_82->setText(QApplication::translate("ImageProcessingSettingsDialog", "[0-999]", 0, QApplication::UnicodeUTF8));
        label_83->setText(QApplication::translate("ImageProcessingSettingsDialog", "Aperture Size:", 0, QApplication::UnicodeUTF8));
        label_84->setText(QApplication::translate("ImageProcessingSettingsDialog", "[3/5/7]", 0, QApplication::UnicodeUTF8));
        cannyL2NormCheckBox->setText(QApplication::translate("ImageProcessingSettingsDialog", "Use L2-norm", 0, QApplication::UnicodeUTF8));
        resetCannyToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset to Defaults", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(cannyTab_5), QApplication::translate("ImageProcessingSettingsDialog", "Canny", 0, QApplication::UnicodeUTF8));
        applyButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Apply", 0, QApplication::UnicodeUTF8));
        resetAllToDefaultsButton->setText(QApplication::translate("ImageProcessingSettingsDialog", "Reset All to Defaults", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ImageProcessingSettingsDialog: public Ui_ImageProcessingSettingsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMAGEPROCESSINGSETTINGSDIALOG_H
