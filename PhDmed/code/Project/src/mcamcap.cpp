#include "mcamcap.h"
#include "ui_mcamcap.h"
#include "calibratedialog.h"
#include "uvccapture.h"

mCamCap::mCamCap(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::mCamCap)
{
    ui->setupUi(this);
    mThread=new CaptureThread(160,120);
    connect(mThread,SIGNAL(imageAquired(bool)),this,SLOT(onImageAquired(bool)));


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

     mThread->start();
    // qDebug()<<"capture\n\n\n\n**************************************************************************";
    //    QMutex mutex;
    //    mutex.lock();
    //    if (mThread->stopped==true){
    //        mThread->start();
    //        mThread->stopped=false;
    //        ui->cmdCapture->setText("Resume");
    //    }else{
    //        mThread->stop();
    //        mThread->stopped=true;
    //        ui->cmdCapture->setText("Pause");
    //    }
    //    mutex.unlock();
}

void mCamCap::onImageAquired(bool aqr){
   // QMutex mutex;
   // mutex.lock();
   // mThread->stop();
    if (aqr){
        mThread->stop();
            ui->lblCam_0->setPixmap(QPixmap::fromImage(mThread->qimg[0]));
            ui->lblCam_1->setPixmap(QPixmap::fromImage(mThread->getQimg(1)));
            ui->lblCam_2->setPixmap(QPixmap::fromImage(mThread->getQimg(2)));
            ui->lblCam_3->setPixmap(QPixmap::fromImage(mThread->getQimg(3)));
            ui->lblCam_4->setPixmap(QPixmap::fromImage(mThread->getQimg(4)));
            ui->lblCam_5->setPixmap(QPixmap::fromImage(mThread->getQimg(5)));
            ui->lblCam_6->setPixmap(QPixmap::fromImage(mThread->getQimg(6)));
            ui->lblCam_7->setPixmap(QPixmap::fromImage(mThread->getQimg(7)));
    }
    //mThread->start();
   // mutex.unlock();
}
void mCamCap::createMenus()
{
    calibrateMenu = menuBar()->addMenu(tr("&Calibrate"));
    calibrateMenu->addAction(calibrateAction);

}
void mCamCap::calibrate(){
calibrateDialog dialog(this);
dialog.exec();
}
