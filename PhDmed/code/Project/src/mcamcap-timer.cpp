#include "mcamcap.h"
#include "ui_mcamcap.h"

#include "uvccapture.h"
#include <QtCore>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
using namespace cv;
mCamCap::mCamCap(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::mCamCap)
{
    ui->setupUi(this);
//ui->gridLayout_2->geometry().setRect(2,2,ui->centralWidget->geometry().width(),ui->centralWidget->geometry().height());

    tmrTimer= new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
    processFrameAndUpdateGUI();
    //start time and set fps to 20
    tmrTimer->start(1);

}

mCamCap::~mCamCap()
{
    delete ui;
}

void mCamCap::on_cmdCapture_clicked()
{
    if (tmrTimer->isActive()==true){
        tmrTimer->stop();
        ui->cmdCapture->setText("Resume");
    }else{
        tmrTimer->start();
        ui->cmdCapture->setText("Pause");
    }
}
void mCamCap::processFrameAndUpdateGUI(){
    int w=320;
    int h=240;
    //cam 0
    // cap.startCapture(0,w,h,224,196,64,128);
    imgCam0=capture(0,w,h,128,32,32,64);
    cvtColor(imgCam0, imgCam0, CV_BGR2RGB);
    qimgCam0= QImage((const unsigned char*)(imgCam0.data),
                     imgCam0.cols,imgCam0.rows,imgCam0.step,QImage::Format_RGB888);

    //cam 1
    imgCam1=capture(1,w,h,128,32,32,64);
    cvtColor(imgCam1, imgCam1, CV_BGR2RGB);
    qimgCam1= QImage((const unsigned char*)(imgCam1.data),
                     imgCam1.cols,imgCam1.rows,imgCam1.step,QImage::Format_RGB888);


    //cam 2
    imgCam2=capture(2,w,h,128,32,32,64);
    cvtColor(imgCam2, imgCam2, CV_BGR2RGB);
    qimgCam2= QImage((const unsigned char*)(imgCam2.data),
                     imgCam2.cols,imgCam2.rows,imgCam2.step,QImage::Format_RGB888);


    //cam 3
    imgCam3=capture(3,w,h,128,32,32,64);
    cvtColor(imgCam3, imgCam3, CV_BGR2RGB);
    qimgCam3= QImage((const unsigned char*)(imgCam3.data),
                     imgCam3.cols,imgCam3.rows,imgCam3.step,QImage::Format_RGB888);

    //cam 4
    imgCam4=capture(4,w,h,128,32,32,64);
    cvtColor(imgCam4, imgCam4, CV_BGR2RGB);
    qimgCam4= QImage((const unsigned char*)(imgCam4.data),
                     imgCam4.cols,imgCam4.rows,imgCam4.step,QImage::Format_RGB888);
    // display on label


    //cam 5
    imgCam5=capture(5,w,h,128,32,32,64);
    cvtColor(imgCam5, imgCam5, CV_BGR2RGB);
    qimgCam5= QImage((const unsigned char*)(imgCam5.data),
                     imgCam5.cols,imgCam5.rows,imgCam5.step,QImage::Format_RGB888);

    //cam 6
    imgCam6=capture(6,w,h,128,32,32,64);
    cvtColor(imgCam6, imgCam6, CV_BGR2RGB);
    qimgCam6= QImage((const unsigned char*)(imgCam6.data),
                     imgCam6.cols,imgCam6.rows,imgCam6.step,QImage::Format_RGB888);


    //cam 7
    imgCam7=capture(7,w,h,128,32,32,64);
    cvtColor(imgCam7, imgCam7, CV_BGR2RGB);
    qimgCam7= QImage((const unsigned char*)(imgCam7.data),
                     imgCam7.cols,imgCam7.rows,imgCam7.step,QImage::Format_RGB888);

    // display on label
    ui->lblCam_0->setPixmap(QPixmap::fromImage(qimgCam0));
    ui->lblCam_1->setPixmap(QPixmap::fromImage(qimgCam1));
    ui->lblCam_2->setPixmap(QPixmap::fromImage(qimgCam2));
    ui->lblCam_3->setPixmap(QPixmap::fromImage(qimgCam3));
    ui->lblCam_4->setPixmap(QPixmap::fromImage(qimgCam4));
    ui->lblCam_5->setPixmap(QPixmap::fromImage(qimgCam5));
    ui->lblCam_6->setPixmap(QPixmap::fromImage(qimgCam6));
    ui->lblCam_7->setPixmap(QPixmap::fromImage(qimgCam7));
}
