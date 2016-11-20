/********************************************************************************
** Form generated from reading UI file 'CameraView.ui'
**
** Created: Thu May 23 12:17:14 2013
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERAVIEW_H
#define UI_CAMERAVIEW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "FrameLabel.h"

QT_BEGIN_NAMESPACE

class Ui_CameraView
{
public:
    QAction *actionGrayscale;
    QAction *actionSmooth;
    QAction *actionDilate;
    QAction *actionErode;
    QAction *actionFlip;
    QAction *actionCanny;
    QAction *imageProcessingSettingsAction;
    QAction *actionScaleToFitFrame;
    QWidget *centralwidget;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_2;
    FrameLabel *frameLabel;
    QFrame *line;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_4;
    QLabel *deviceNumberLabel;
    QSpacerItem *horizontalSpacer_5;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_5;
    QLabel *cameraResolutionLabel;
    QSpacerItem *horizontalSpacer_6;
    QHBoxLayout *horizontalLayout;
    QLabel *label_1;
    QProgressBar *imageBufferBar;
    QLabel *imageBufferLabel;
    QSpacerItem *horizontalSpacer_7;
    QPushButton *clearImageBufferButton;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_2;
    QLabel *captureRateLabel;
    QLabel *nFramesCapturedLabel;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_3;
    QLabel *processingRateLabel;
    QLabel *nFramesProcessedLabel;
    QSpacerItem *horizontalSpacer_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QLabel *roiLabel;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_7;
    QLabel *mouseCursorPosLabel;
    QSpacerItem *horizontalSpacer;
    QFrame *line_2;
    QMenuBar *menubar;
    QMenu *imageProcessingMenu;
    QMenu *menuOptions;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *CameraView)
    {
        if (CameraView->objectName().isEmpty())
            CameraView->setObjectName(QString::fromUtf8("CameraView"));
        CameraView->resize(660, 690);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(CameraView->sizePolicy().hasHeightForWidth());
        CameraView->setSizePolicy(sizePolicy);
        CameraView->setMinimumSize(QSize(660, 690));
        actionGrayscale = new QAction(CameraView);
        actionGrayscale->setObjectName(QString::fromUtf8("actionGrayscale"));
        actionGrayscale->setCheckable(true);
        actionSmooth = new QAction(CameraView);
        actionSmooth->setObjectName(QString::fromUtf8("actionSmooth"));
        actionSmooth->setCheckable(true);
        actionDilate = new QAction(CameraView);
        actionDilate->setObjectName(QString::fromUtf8("actionDilate"));
        actionDilate->setCheckable(true);
        actionErode = new QAction(CameraView);
        actionErode->setObjectName(QString::fromUtf8("actionErode"));
        actionErode->setCheckable(true);
        actionFlip = new QAction(CameraView);
        actionFlip->setObjectName(QString::fromUtf8("actionFlip"));
        actionFlip->setCheckable(true);
        actionCanny = new QAction(CameraView);
        actionCanny->setObjectName(QString::fromUtf8("actionCanny"));
        actionCanny->setCheckable(true);
        imageProcessingSettingsAction = new QAction(CameraView);
        imageProcessingSettingsAction->setObjectName(QString::fromUtf8("imageProcessingSettingsAction"));
        actionScaleToFitFrame = new QAction(CameraView);
        actionScaleToFitFrame->setObjectName(QString::fromUtf8("actionScaleToFitFrame"));
        actionScaleToFitFrame->setCheckable(true);
        centralwidget = new QWidget(CameraView);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        sizePolicy.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy);
        centralwidget->setMinimumSize(QSize(660, 643));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 641, 623));
        verticalLayout_2 = new QVBoxLayout(layoutWidget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        frameLabel = new FrameLabel(layoutWidget);
        frameLabel->setObjectName(QString::fromUtf8("frameLabel"));
        frameLabel->setEnabled(true);
        sizePolicy.setHeightForWidth(frameLabel->sizePolicy().hasHeightForWidth());
        frameLabel->setSizePolicy(sizePolicy);
        frameLabel->setMinimumSize(QSize(640, 480));
        frameLabel->setMaximumSize(QSize(640, 480));
        frameLabel->setMouseTracking(true);
        frameLabel->setAutoFillBackground(true);
        frameLabel->setFrameShape(QFrame::Box);
        frameLabel->setScaledContents(false);
        frameLabel->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(frameLabel);

        line = new QFrame(layoutWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);
        label_4->setMinimumSize(QSize(180, 0));
        QFont font;
        font.setPointSize(8);
        font.setBold(true);
        font.setWeight(75);
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(label_4);

        deviceNumberLabel = new QLabel(layoutWidget);
        deviceNumberLabel->setObjectName(QString::fromUtf8("deviceNumberLabel"));
        sizePolicy.setHeightForWidth(deviceNumberLabel->sizePolicy().hasHeightForWidth());
        deviceNumberLabel->setSizePolicy(sizePolicy);
        deviceNumberLabel->setMinimumSize(QSize(180, 0));
        QFont font1;
        font1.setPointSize(8);
        deviceNumberLabel->setFont(font1);
        deviceNumberLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(deviceNumberLabel);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_5);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);
        label_5->setMinimumSize(QSize(180, 0));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_3->addWidget(label_5);

        cameraResolutionLabel = new QLabel(layoutWidget);
        cameraResolutionLabel->setObjectName(QString::fromUtf8("cameraResolutionLabel"));
        sizePolicy.setHeightForWidth(cameraResolutionLabel->sizePolicy().hasHeightForWidth());
        cameraResolutionLabel->setSizePolicy(sizePolicy);
        cameraResolutionLabel->setMinimumSize(QSize(180, 0));
        cameraResolutionLabel->setFont(font1);
        cameraResolutionLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_3->addWidget(cameraResolutionLabel);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_6);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_1 = new QLabel(layoutWidget);
        label_1->setObjectName(QString::fromUtf8("label_1"));
        sizePolicy.setHeightForWidth(label_1->sizePolicy().hasHeightForWidth());
        label_1->setSizePolicy(sizePolicy);
        label_1->setMinimumSize(QSize(180, 0));
        label_1->setFont(font);
        label_1->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout->addWidget(label_1);

        imageBufferBar = new QProgressBar(layoutWidget);
        imageBufferBar->setObjectName(QString::fromUtf8("imageBufferBar"));
        sizePolicy.setHeightForWidth(imageBufferBar->sizePolicy().hasHeightForWidth());
        imageBufferBar->setSizePolicy(sizePolicy);
        imageBufferBar->setMinimumSize(QSize(120, 0));
        imageBufferBar->setFont(font1);
        imageBufferBar->setValue(0);

        horizontalLayout->addWidget(imageBufferBar);

        imageBufferLabel = new QLabel(layoutWidget);
        imageBufferLabel->setObjectName(QString::fromUtf8("imageBufferLabel"));
        sizePolicy.setHeightForWidth(imageBufferLabel->sizePolicy().hasHeightForWidth());
        imageBufferLabel->setSizePolicy(sizePolicy);
        imageBufferLabel->setMinimumSize(QSize(60, 0));
        imageBufferLabel->setFont(font1);
        imageBufferLabel->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(imageBufferLabel);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_7);


        verticalLayout->addLayout(horizontalLayout);

        clearImageBufferButton = new QPushButton(layoutWidget);
        clearImageBufferButton->setObjectName(QString::fromUtf8("clearImageBufferButton"));
        sizePolicy.setHeightForWidth(clearImageBufferButton->sizePolicy().hasHeightForWidth());
        clearImageBufferButton->setSizePolicy(sizePolicy);
        clearImageBufferButton->setMinimumSize(QSize(360, 0));
        clearImageBufferButton->setFont(font1);

        verticalLayout->addWidget(clearImageBufferButton);


        horizontalLayout_8->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setMinimumSize(QSize(110, 0));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_7->addWidget(label_2);

        captureRateLabel = new QLabel(layoutWidget);
        captureRateLabel->setObjectName(QString::fromUtf8("captureRateLabel"));
        captureRateLabel->setEnabled(true);
        sizePolicy.setHeightForWidth(captureRateLabel->sizePolicy().hasHeightForWidth());
        captureRateLabel->setSizePolicy(sizePolicy);
        captureRateLabel->setMinimumSize(QSize(70, 0));
        captureRateLabel->setFont(font1);

        horizontalLayout_7->addWidget(captureRateLabel);

        nFramesCapturedLabel = new QLabel(layoutWidget);
        nFramesCapturedLabel->setObjectName(QString::fromUtf8("nFramesCapturedLabel"));
        sizePolicy.setHeightForWidth(nFramesCapturedLabel->sizePolicy().hasHeightForWidth());
        nFramesCapturedLabel->setSizePolicy(sizePolicy);
        nFramesCapturedLabel->setMinimumSize(QSize(70, 0));
        nFramesCapturedLabel->setFont(font1);

        horizontalLayout_7->addWidget(nFramesCapturedLabel);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_4);


        verticalLayout_3->addLayout(horizontalLayout_7);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setMinimumSize(QSize(110, 0));
        label_3->setFont(font);

        horizontalLayout_6->addWidget(label_3);

        processingRateLabel = new QLabel(layoutWidget);
        processingRateLabel->setObjectName(QString::fromUtf8("processingRateLabel"));
        sizePolicy.setHeightForWidth(processingRateLabel->sizePolicy().hasHeightForWidth());
        processingRateLabel->setSizePolicy(sizePolicy);
        processingRateLabel->setMinimumSize(QSize(70, 0));
        processingRateLabel->setFont(font1);

        horizontalLayout_6->addWidget(processingRateLabel);

        nFramesProcessedLabel = new QLabel(layoutWidget);
        nFramesProcessedLabel->setObjectName(QString::fromUtf8("nFramesProcessedLabel"));
        sizePolicy.setHeightForWidth(nFramesProcessedLabel->sizePolicy().hasHeightForWidth());
        nFramesProcessedLabel->setSizePolicy(sizePolicy);
        nFramesProcessedLabel->setMinimumSize(QSize(70, 0));
        nFramesProcessedLabel->setFont(font1);

        horizontalLayout_6->addWidget(nFramesProcessedLabel);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setMinimumSize(QSize(110, 0));
        label_6->setFont(font);

        horizontalLayout_4->addWidget(label_6);

        roiLabel = new QLabel(layoutWidget);
        roiLabel->setObjectName(QString::fromUtf8("roiLabel"));
        sizePolicy.setHeightForWidth(roiLabel->sizePolicy().hasHeightForWidth());
        roiLabel->setSizePolicy(sizePolicy);
        roiLabel->setMinimumSize(QSize(140, 0));
        roiLabel->setFont(font1);

        horizontalLayout_4->addWidget(roiLabel);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);
        label_7->setMinimumSize(QSize(110, 0));
        label_7->setFont(font);

        horizontalLayout_5->addWidget(label_7);

        mouseCursorPosLabel = new QLabel(layoutWidget);
        mouseCursorPosLabel->setObjectName(QString::fromUtf8("mouseCursorPosLabel"));
        sizePolicy.setHeightForWidth(mouseCursorPosLabel->sizePolicy().hasHeightForWidth());
        mouseCursorPosLabel->setSizePolicy(sizePolicy);
        mouseCursorPosLabel->setMinimumSize(QSize(140, 0));
        mouseCursorPosLabel->setFont(font1);

        horizontalLayout_5->addWidget(mouseCursorPosLabel);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_5);


        horizontalLayout_8->addLayout(verticalLayout_3);


        verticalLayout_2->addLayout(horizontalLayout_8);

        line_2 = new QFrame(layoutWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        CameraView->setCentralWidget(centralwidget);
        menubar = new QMenuBar(CameraView);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 660, 25));
        imageProcessingMenu = new QMenu(menubar);
        imageProcessingMenu->setObjectName(QString::fromUtf8("imageProcessingMenu"));
        menuOptions = new QMenu(menubar);
        menuOptions->setObjectName(QString::fromUtf8("menuOptions"));
        CameraView->setMenuBar(menubar);
        statusbar = new QStatusBar(CameraView);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        CameraView->setStatusBar(statusbar);

        menubar->addAction(imageProcessingMenu->menuAction());
        menubar->addAction(menuOptions->menuAction());
        imageProcessingMenu->addAction(actionGrayscale);
        imageProcessingMenu->addAction(actionSmooth);
        imageProcessingMenu->addAction(actionDilate);
        imageProcessingMenu->addAction(actionErode);
        imageProcessingMenu->addAction(actionFlip);
        imageProcessingMenu->addAction(actionCanny);
        imageProcessingMenu->addSeparator();
        imageProcessingMenu->addAction(imageProcessingSettingsAction);
        menuOptions->addAction(actionScaleToFitFrame);

        retranslateUi(CameraView);

        QMetaObject::connectSlotsByName(CameraView);
    } // setupUi

    void retranslateUi(QMainWindow *CameraView)
    {
        CameraView->setWindowTitle(QApplication::translate("CameraView", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionGrayscale->setText(QApplication::translate("CameraView", "1: Grayscale", 0, QApplication::UnicodeUTF8));
        actionSmooth->setText(QApplication::translate("CameraView", "2. Smooth", 0, QApplication::UnicodeUTF8));
        actionDilate->setText(QApplication::translate("CameraView", "3. Dilate", 0, QApplication::UnicodeUTF8));
        actionErode->setText(QApplication::translate("CameraView", "4. Erode", 0, QApplication::UnicodeUTF8));
        actionFlip->setText(QApplication::translate("CameraView", "5. Flip", 0, QApplication::UnicodeUTF8));
        actionCanny->setText(QApplication::translate("CameraView", "6. Canny", 0, QApplication::UnicodeUTF8));
        imageProcessingSettingsAction->setText(QApplication::translate("CameraView", "Settings...", 0, QApplication::UnicodeUTF8));
        actionScaleToFitFrame->setText(QApplication::translate("CameraView", "Scale to fit frame", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("CameraView", "Camera Device Number:", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("CameraView", "Camera Resolution:", 0, QApplication::UnicodeUTF8));
        label_1->setText(QApplication::translate("CameraView", "Image Buffer Status (% full):", 0, QApplication::UnicodeUTF8));
        imageBufferLabel->setText(QString());
        clearImageBufferButton->setText(QApplication::translate("CameraView", "Clear Image Buffer", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("CameraView", "Capture Rate:", 0, QApplication::UnicodeUTF8));
        captureRateLabel->setText(QString());
        nFramesCapturedLabel->setText(QString());
        label_3->setText(QApplication::translate("CameraView", "Processing Rate:", 0, QApplication::UnicodeUTF8));
        processingRateLabel->setText(QString());
        nFramesProcessedLabel->setText(QString());
        label_6->setText(QApplication::translate("CameraView", "ROI:", 0, QApplication::UnicodeUTF8));
        roiLabel->setText(QString());
        label_7->setText(QApplication::translate("CameraView", "Cursor:", 0, QApplication::UnicodeUTF8));
        mouseCursorPosLabel->setText(QString());
        imageProcessingMenu->setTitle(QApplication::translate("CameraView", "Image Processing", 0, QApplication::UnicodeUTF8));
        menuOptions->setTitle(QApplication::translate("CameraView", "Options", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CameraView: public Ui_CameraView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERAVIEW_H
