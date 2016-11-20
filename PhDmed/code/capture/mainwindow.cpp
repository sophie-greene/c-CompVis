#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "capturevideo.h"
#include "uvccapture.h"
#include <QtCore>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
using namespace cv;
MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int w=340;
    int h=260;

    ui->lblCam_0->setGeometry(2,2,w,h);
    ui->lblCam_1->setGeometry(w,2,w,h);
    ui->lblCam_2->setGeometry(2,h,w,h);
    ui->lblCam_3->setGeometry(w,h,w,h);
    ui->lblCam_4->setGeometry(2,2*h,w,h);
    ui->lblCam_5->setGeometry(w,2*h,w,h);
    ui->lblCam_6->setGeometry(2*w,2,w,h);
    ui->lblCam_7->setGeometry(2*w,h,w,h);

    tmrTimer= new QTimer(this);
    connect(tmrTimer,SIGNAL(timeout()),this,SLOT(processFrameAndUpdateGUI()));
    processFrameAndUpdateGUI();
    //start time and set fps to 20
    tmrTimer->start(20);
}

MainWindow::~MainWindow()
{
    delete ui;
    tmrTimer->stop();

}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_btnShow_clicked()
{
    if (tmrTimer->isActive()==true){
        tmrTimer->stop();
        ui->btnShow->setText("Resume");
    }else{
        tmrTimer->start();
        ui->btnShow->setText("Pause");
    }



}
void MainWindow::processFrameAndUpdateGUI(){
    int w=320;
    int h=240;
    //cam 0
    //CaptureVideo cap;


   // cap.startCapture(0,w,h,224,196,64,128);
    imgCam0=capture(0,w,h,0,0,0,0);
    cvtColor(imgCam0, imgCam0, CV_BGR2RGB);
    qimgCam0= QImage((const unsigned char*)(imgCam0.data),
                     imgCam0.cols,imgCam0.rows,imgCam0.step,QImage::Format_RGB888);
    // display on label
    ui->lblCam_0->setPixmap(QPixmap::fromImage(qimgCam0));

    //cam 1
    imgCam1=capture(1,w,h,224,196,0,0);
    cvtColor(imgCam1, imgCam1, CV_BGR2RGB);
    qimgCam1= QImage((const unsigned char*)(imgCam1.data),
                     imgCam1.cols,imgCam1.rows,imgCam1.step,QImage::Format_RGB888);


    //cam 2
    imgCam2=capture(2,w,h,224,196,0,0);
    cvtColor(imgCam2, imgCam2, CV_BGR2RGB);
    qimgCam2= QImage((const unsigned char*)(imgCam2.data),
                     imgCam2.cols,imgCam2.rows,imgCam2.step,QImage::Format_RGB888);


    //cam 3
    imgCam3=capture(3,w,h,0,0,0,0);
    cvtColor(imgCam3, imgCam3, CV_BGR2RGB);
    qimgCam3= QImage((const unsigned char*)(imgCam3.data),
                     imgCam3.cols,imgCam3.rows,imgCam3.step,QImage::Format_RGB888);

    //cam 4
    imgCam4=capture(4,w,h,0,0,0,0);
    cvtColor(imgCam4, imgCam4, CV_BGR2RGB);
    qimgCam4= QImage((const unsigned char*)(imgCam4.data),
                     imgCam4.cols,imgCam4.rows,imgCam4.step,QImage::Format_RGB888);
    // display on label


    //cam 5
    imgCam5=capture(5,w,h,0,0,0,0);
    cvtColor(imgCam5, imgCam5, CV_BGR2RGB);
    qimgCam5= QImage((const unsigned char*)(imgCam5.data),
                     imgCam5.cols,imgCam5.rows,imgCam5.step,QImage::Format_RGB888);

    //cam 6
    imgCam6=capture(6,w,h,0,0,0,0);
    cvtColor(imgCam6, imgCam6, CV_BGR2RGB);
    qimgCam6= QImage((const unsigned char*)(imgCam6.data),
                     imgCam6.cols,imgCam6.rows,imgCam6.step,QImage::Format_RGB888);


    //cam 7
    imgCam7=capture(7,w,h,0,0,0,0);
    cvtColor(imgCam7, imgCam7, CV_BGR2RGB);
    qimgCam7= QImage((const unsigned char*)(imgCam7.data),
                     imgCam7.cols,imgCam7.rows,imgCam7.step,QImage::Format_RGB888);

    // display on label
    ui->lblCam_1->setPixmap(QPixmap::fromImage(qimgCam1));
    ui->lblCam_2->setPixmap(QPixmap::fromImage(qimgCam2));
    ui->lblCam_3->setPixmap(QPixmap::fromImage(qimgCam3));
    ui->lblCam_4->setPixmap(QPixmap::fromImage(qimgCam4));
    ui->lblCam_5->setPixmap(QPixmap::fromImage(qimgCam5));
    ui->lblCam_6->setPixmap(QPixmap::fromImage(qimgCam6));
    ui->lblCam_7->setPixmap(QPixmap::fromImage(qimgCam7));
}
