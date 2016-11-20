/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created: Fri Dec 7 10:21:54 2012
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
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "FrameLabel.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *connectToCameraAction;
    QAction *disconnectCameraAction;
    QAction *exitAction;
    QAction *aboutAction;
    QAction *grayscaleAction;
    QAction *smoothAction;
    QAction *flipAction;
    QAction *dilateAction;
    QAction *erodeAction;
    QAction *cannyAction;
    QAction *imageProcessingSettingsAction;
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
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_5;
    QLabel *cameraResolutionLabel;
    QHBoxLayout *horizontalLayout;
    QLabel *label_1;
    QProgressBar *imageBufferBar;
    QLabel *imageBufferLabel;
    QPushButton *clearImageBufferButton;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_2;
    QLabel *captureRateLabel;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_3;
    QLabel *processingRateLabel;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QLabel *roiLabel;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_7;
    QLabel *mouseCursorPosLabel;
    QFrame *line_2;
    QMenuBar *menubar;
    QMenu *mainMenu;
    QMenu *aboutMenu;
    QMenu *imageProcessingMenu;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(671, 654);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(661, 633));
        MainWindow->setAutoFillBackground(false);
        connectToCameraAction = new QAction(MainWindow);
        connectToCameraAction->setObjectName(QString::fromUtf8("connectToCameraAction"));
        disconnectCameraAction = new QAction(MainWindow);
        disconnectCameraAction->setObjectName(QString::fromUtf8("disconnectCameraAction"));
        exitAction = new QAction(MainWindow);
        exitAction->setObjectName(QString::fromUtf8("exitAction"));
        aboutAction = new QAction(MainWindow);
        aboutAction->setObjectName(QString::fromUtf8("aboutAction"));
        grayscaleAction = new QAction(MainWindow);
        grayscaleAction->setObjectName(QString::fromUtf8("grayscaleAction"));
        grayscaleAction->setCheckable(true);
        smoothAction = new QAction(MainWindow);
        smoothAction->setObjectName(QString::fromUtf8("smoothAction"));
        smoothAction->setCheckable(true);
        flipAction = new QAction(MainWindow);
        flipAction->setObjectName(QString::fromUtf8("flipAction"));
        flipAction->setCheckable(true);
        dilateAction = new QAction(MainWindow);
        dilateAction->setObjectName(QString::fromUtf8("dilateAction"));
        dilateAction->setCheckable(true);
        erodeAction = new QAction(MainWindow);
        erodeAction->setObjectName(QString::fromUtf8("erodeAction"));
        erodeAction->setCheckable(true);
        cannyAction = new QAction(MainWindow);
        cannyAction->setObjectName(QString::fromUtf8("cannyAction"));
        cannyAction->setCheckable(true);
        imageProcessingSettingsAction = new QAction(MainWindow);
        imageProcessingSettingsAction->setObjectName(QString::fromUtf8("imageProcessingSettingsAction"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 651, 611));
        verticalLayout_2 = new QVBoxLayout(layoutWidget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        frameLabel = new FrameLabel(layoutWidget);
        frameLabel->setObjectName(QString::fromUtf8("frameLabel"));
        frameLabel->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(frameLabel->sizePolicy().hasHeightForWidth());
        frameLabel->setSizePolicy(sizePolicy1);
        frameLabel->setMinimumSize(QSize(642, 482));
        frameLabel->setMouseTracking(true);
        frameLabel->setAutoFillBackground(true);
        frameLabel->setFrameShape(QFrame::Box);
        frameLabel->setFrameShadow(QFrame::Raised);
        frameLabel->setLineWidth(1);
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
        label_4->setMinimumSize(QSize(180, 0));
        label_4->setMaximumSize(QSize(180, 16777215));
        QFont font;
        font.setPointSize(8);
        font.setBold(true);
        font.setWeight(75);
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(label_4);

        deviceNumberLabel = new QLabel(layoutWidget);
        deviceNumberLabel->setObjectName(QString::fromUtf8("deviceNumberLabel"));
        QFont font1;
        font1.setPointSize(8);
        deviceNumberLabel->setFont(font1);
        deviceNumberLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(deviceNumberLabel);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setMinimumSize(QSize(180, 0));
        label_5->setMaximumSize(QSize(180, 16777215));
        label_5->setFont(font);
        label_5->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_3->addWidget(label_5);

        cameraResolutionLabel = new QLabel(layoutWidget);
        cameraResolutionLabel->setObjectName(QString::fromUtf8("cameraResolutionLabel"));
        cameraResolutionLabel->setFont(font1);
        cameraResolutionLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_3->addWidget(cameraResolutionLabel);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_1 = new QLabel(layoutWidget);
        label_1->setObjectName(QString::fromUtf8("label_1"));
        label_1->setMinimumSize(QSize(180, 0));
        label_1->setMaximumSize(QSize(180, 16777215));
        label_1->setFont(font);
        label_1->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout->addWidget(label_1);

        imageBufferBar = new QProgressBar(layoutWidget);
        imageBufferBar->setObjectName(QString::fromUtf8("imageBufferBar"));
        imageBufferBar->setFont(font1);
        imageBufferBar->setValue(0);

        horizontalLayout->addWidget(imageBufferBar);

        imageBufferLabel = new QLabel(layoutWidget);
        imageBufferLabel->setObjectName(QString::fromUtf8("imageBufferLabel"));
        imageBufferLabel->setMinimumSize(QSize(60, 0));
        imageBufferLabel->setMaximumSize(QSize(60, 16777215));
        imageBufferLabel->setFont(font1);
        imageBufferLabel->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(imageBufferLabel);


        verticalLayout->addLayout(horizontalLayout);

        clearImageBufferButton = new QPushButton(layoutWidget);
        clearImageBufferButton->setObjectName(QString::fromUtf8("clearImageBufferButton"));
        clearImageBufferButton->setFont(font1);

        verticalLayout->addWidget(clearImageBufferButton);


        horizontalLayout_8->addLayout(verticalLayout);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMinimumSize(QSize(110, 0));
        label_2->setMaximumSize(QSize(110, 16777215));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_7->addWidget(label_2);

        captureRateLabel = new QLabel(layoutWidget);
        captureRateLabel->setObjectName(QString::fromUtf8("captureRateLabel"));
        captureRateLabel->setEnabled(true);
        captureRateLabel->setMinimumSize(QSize(140, 0));
        captureRateLabel->setMaximumSize(QSize(140, 16777215));
        captureRateLabel->setFont(font1);
        captureRateLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_7->addWidget(captureRateLabel);


        verticalLayout_3->addLayout(horizontalLayout_7);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(110, 0));
        label_3->setMaximumSize(QSize(110, 16777215));
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_6->addWidget(label_3);

        processingRateLabel = new QLabel(layoutWidget);
        processingRateLabel->setObjectName(QString::fromUtf8("processingRateLabel"));
        processingRateLabel->setMinimumSize(QSize(140, 0));
        processingRateLabel->setMaximumSize(QSize(140, 16777215));
        processingRateLabel->setFont(font1);
        processingRateLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout_6->addWidget(processingRateLabel);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setMinimumSize(QSize(110, 0));
        label_6->setMaximumSize(QSize(110, 16777215));
        label_6->setFont(font);

        horizontalLayout_4->addWidget(label_6);

        roiLabel = new QLabel(layoutWidget);
        roiLabel->setObjectName(QString::fromUtf8("roiLabel"));
        roiLabel->setMinimumSize(QSize(140, 0));
        roiLabel->setMaximumSize(QSize(140, 16777215));
        roiLabel->setFont(font1);

        horizontalLayout_4->addWidget(roiLabel);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setMinimumSize(QSize(110, 0));
        label_7->setMaximumSize(QSize(110, 16777215));
        label_7->setFont(font);

        horizontalLayout_5->addWidget(label_7);

        mouseCursorPosLabel = new QLabel(layoutWidget);
        mouseCursorPosLabel->setObjectName(QString::fromUtf8("mouseCursorPosLabel"));
        mouseCursorPosLabel->setMinimumSize(QSize(140, 0));
        mouseCursorPosLabel->setMaximumSize(QSize(140, 16777215));
        mouseCursorPosLabel->setFont(font1);

        horizontalLayout_5->addWidget(mouseCursorPosLabel);


        verticalLayout_3->addLayout(horizontalLayout_5);


        horizontalLayout_8->addLayout(verticalLayout_3);


        verticalLayout_2->addLayout(horizontalLayout_8);

        line_2 = new QFrame(layoutWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 671, 23));
        mainMenu = new QMenu(menubar);
        mainMenu->setObjectName(QString::fromUtf8("mainMenu"));
        aboutMenu = new QMenu(menubar);
        aboutMenu->setObjectName(QString::fromUtf8("aboutMenu"));
        imageProcessingMenu = new QMenu(menubar);
        imageProcessingMenu->setObjectName(QString::fromUtf8("imageProcessingMenu"));
        MainWindow->setMenuBar(menubar);

        menubar->addAction(mainMenu->menuAction());
        menubar->addAction(imageProcessingMenu->menuAction());
        menubar->addAction(aboutMenu->menuAction());
        mainMenu->addAction(connectToCameraAction);
        mainMenu->addAction(disconnectCameraAction);
        mainMenu->addSeparator();
        mainMenu->addAction(exitAction);
        aboutMenu->addAction(aboutAction);
        imageProcessingMenu->addAction(grayscaleAction);
        imageProcessingMenu->addAction(smoothAction);
        imageProcessingMenu->addAction(dilateAction);
        imageProcessingMenu->addAction(erodeAction);
        imageProcessingMenu->addAction(flipAction);
        imageProcessingMenu->addAction(cannyAction);
        imageProcessingMenu->addSeparator();
        imageProcessingMenu->addAction(imageProcessingSettingsAction);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "qt-opencv-multithreaded", 0, QApplication::UnicodeUTF8));
        connectToCameraAction->setText(QApplication::translate("MainWindow", "Connect to Camera...", 0, QApplication::UnicodeUTF8));
        disconnectCameraAction->setText(QApplication::translate("MainWindow", "Disconnect Camera", 0, QApplication::UnicodeUTF8));
        exitAction->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        aboutAction->setText(QApplication::translate("MainWindow", "About...", 0, QApplication::UnicodeUTF8));
        grayscaleAction->setText(QApplication::translate("MainWindow", "1: Grayscale", 0, QApplication::UnicodeUTF8));
        smoothAction->setText(QApplication::translate("MainWindow", "2: Smooth", 0, QApplication::UnicodeUTF8));
        flipAction->setText(QApplication::translate("MainWindow", "5: Flip", 0, QApplication::UnicodeUTF8));
        dilateAction->setText(QApplication::translate("MainWindow", "3: Dilate", 0, QApplication::UnicodeUTF8));
        erodeAction->setText(QApplication::translate("MainWindow", "4: Erode", 0, QApplication::UnicodeUTF8));
        cannyAction->setText(QApplication::translate("MainWindow", "6. Canny", 0, QApplication::UnicodeUTF8));
        imageProcessingSettingsAction->setText(QApplication::translate("MainWindow", "Settings...", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Camera Device Number:", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Camera Resolution:", 0, QApplication::UnicodeUTF8));
        label_1->setText(QApplication::translate("MainWindow", "Image Buffer Status (% full):", 0, QApplication::UnicodeUTF8));
        imageBufferLabel->setText(QString());
        clearImageBufferButton->setText(QApplication::translate("MainWindow", "Clear Image Buffer", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Capture Rate:", 0, QApplication::UnicodeUTF8));
        captureRateLabel->setText(QString());
        label_3->setText(QApplication::translate("MainWindow", "Processing Rate:", 0, QApplication::UnicodeUTF8));
        processingRateLabel->setText(QString());
        label_6->setText(QApplication::translate("MainWindow", "ROI:", 0, QApplication::UnicodeUTF8));
        roiLabel->setText(QString());
        label_7->setText(QApplication::translate("MainWindow", "Cursor", 0, QApplication::UnicodeUTF8));
        mouseCursorPosLabel->setText(QString());
        mainMenu->setTitle(QApplication::translate("MainWindow", "Main", 0, QApplication::UnicodeUTF8));
        aboutMenu->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        imageProcessingMenu->setTitle(QApplication::translate("MainWindow", "Image Processing", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
