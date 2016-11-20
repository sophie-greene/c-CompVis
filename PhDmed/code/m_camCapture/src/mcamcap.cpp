#include "mcamcap.h"
#include "ui_mcamcap.h"
#include "calibratedialog.h"
#include "uvccapture.h"
#include <boost/lexical_cast.hpp>
mCamCap::mCamCap(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::mCamCap)
{
    ui->setupUi(this);
    mThread0=new CaptureThread(0,160,120);
    connect(mThread0,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired0(bool)));

    mThread1=new CaptureThread(1,160,120);
    connect(mThread1,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired1(bool)));

    mThread2=new CaptureThread(2,160,120);
    connect(mThread2,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired2(bool)));

    mThread3=new CaptureThread(3,160,120);
    connect(mThread3,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired3(bool)));

    mThread4=new CaptureThread(4,160,120);
    connect(mThread4,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired4(bool)));

    mThread5=new CaptureThread(5,160,120);
    connect(mThread5,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired5(bool)));

    mThread6=new CaptureThread(6,160,120);
    connect(mThread6,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired6(bool)));

    mThread7=new CaptureThread(7,160,120);
    connect(mThread7,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired7(bool)));
    //    tmrTimer= new QTimer(this);
    //    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(updateGUI()));
    //
    //    //start time and set fps to 20
    //   tmrTimer->start(1000);
}

mCamCap::~mCamCap()
{
    //mThread->stop();
    delete ui;
}

void mCamCap::on_cmdCapture_clicked()
{

    mThread0->start();

}

void mCamCap::onImageAquired0(bool aqr){


    if (aqr){

            ui->lblCam_0->setPixmap(QPixmap::fromImage(mThread0->getQimg()));
    }
    mThread0->stop();
    mThread1->start();

}

void mCamCap::onImageAquired1(bool aqr){
    if (aqr){
            ui->lblCam_1->setPixmap(QPixmap::fromImage(mThread1->getQimg()));
    }
    mThread1->stop();;
    mThread2->start();
}

void mCamCap::onImageAquired2(bool aqr){
    if (aqr){
            ui->lblCam_2->setPixmap(QPixmap::fromImage(mThread2->getQimg()));
    }
    mThread2->stop();;
    mThread3->start();
}

void mCamCap::onImageAquired3(bool aqr){
    if (aqr){
            ui->lblCam_3->setPixmap(QPixmap::fromImage(mThread3->getQimg()));
    }
    mThread3->stop();;
    mThread4->start();
}

void mCamCap::onImageAquired4(bool aqr){
    if (aqr){
            ui->lblCam_4->setPixmap(QPixmap::fromImage(mThread4->getQimg()));
    }
    mThread4->stop();;
    mThread5->start();
}

void mCamCap::onImageAquired5(bool aqr){
    if (aqr){
            ui->lblCam_5->setPixmap(QPixmap::fromImage(mThread5->getQimg()));
    }
    mThread5->stop();;
    mThread6->start();
}

void mCamCap::onImageAquired6(bool aqr){
    if (aqr){
            ui->lblCam_6->setPixmap(QPixmap::fromImage(mThread6->getQimg()));
    }
    mThread6->stop();;
    mThread7->start();
}

void mCamCap::onImageAquired7(bool aqr){
    if (aqr){
            ui->lblCam_7->setPixmap(QPixmap::fromImage(mThread7->getQimg()));
    }
    mThread7->stop();;
    mThread0->start();
}



void mCamCap::createMenus()
{
    calibrateMenu = menuBar()->addMenu(tr("&Calibrate"));
    calibrateMenu->addAction(calibrateAction);

}
void mCamCap::calibrate(){

}

void mCamCap::on_cmdCalibrate_clicked()
{
    calibrateDialog dialog(this);

    dialog.exec();

}
