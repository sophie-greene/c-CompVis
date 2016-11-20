#include "mainwindow.h"
//#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent)

{
    cmdQuit=new  QPushButton (tr("&Quit"));
    cmdStartCameras=new  QPushButton (tr("&Start Cameras"));
    cmdCalibrate=new  QPushButton (tr("&Calibrate"));
    connect(cmdCalibrate ,
            SIGNAL(clicked()),
            this,
            SLOT(runCalibrate()));
    connect(cmdStartCameras ,
            SIGNAL(clicked()),
            this,
            SLOT(on_cmdCapture_clicked()));
    connect(cmdQuit ,
            SIGNAL(clicked()),
            this,
            SLOT(on_cmdQuit_clicked()));
    widgetResize();
    areCamerasRunning=false;
    isCalibrating=false;

}
void MainWindow::widgetResize()
{
    //get screen resolution
    int w=40*(QApplication::desktop()->availableGeometry().size().width())/100;
    int h=40*(QApplication::desktop()->availableGeometry().size().height())/100;
//    int left=20*(QApplication::desktop()->availableGeometry().size().width())/100;
//    int top=5*(QApplication::desktop()->availableGeometry().size().height())/100;
    int camFrameWidth=(w/3)-(w/20);
    int camFrameHeight=(h/3)-(h/20);

    int leftRightMargins=(w-(camFrameWidth*3))/4;
    int topBottomMargins=(h-(camFrameHeight*3))/4;

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setContentsMargins(leftRightMargins,
                                   topBottomMargins,
                                   leftRightMargins,
                                 topBottomMargins);
  //positioning labels
    int row=0;
    int col=0;
    for(int i=0;i<8;i++){
        if(i%3==0 && i!=0){
            row=row+2;
            col=0;
        }
        lblCam[i]=new QLabel();
        lblCam[i]->setScaledContents(true);
        lbl[i]=new QLabel();
        lblCam[i]->setFixedWidth(camFrameWidth);
        lblCam[i]->setFixedHeight(camFrameHeight);
        QString s;
        s="Camera "+s.number(i);
        lbl[i]->setText(s);


        mainLayout->addWidget(lbl[i],row,col);
        mainLayout->addWidget(lblCam[i],row+1,col);
        col=col+1;
        qDebug()<<"i="<<i<<"("<<row<<","<<col<<")";

    }
    QVBoxLayout* s=new QVBoxLayout();
    s->addWidget(cmdStartCameras,1,Qt::AlignCenter);
    s->addWidget(cmdCalibrate,1,Qt::AlignCenter);
    s->addWidget(cmdQuit,1,Qt::AlignCenter);
s->addStretch(3);
    cmdStartCameras->setFixedWidth(camFrameWidth/3);
    cmdCalibrate->setFixedWidth(camFrameWidth/3);
      cmdQuit  ->setFixedWidth(camFrameWidth/3);
      mainLayout->addLayout(s,5,2,1,1,Qt::AlignBottom|Qt::AlignRight);
    QWidget* widget = new QWidget(this);
    widget->setLayout(mainLayout);
   this->setCentralWidget(widget);
//setLayout(mainLayout);


    setWindowTitle(tr("3D Reconstruction "));
}
void MainWindow::displayImage( QImage  img,int device)
{
    lblCam[device]->setPixmap(QPixmap::fromImage(img));
}
void MainWindow::runCameras()
{
    for (int i=0; i<8;i++)
    {
        cThread[i]=new CaptureThread(i,FR_WIDTH,FR_HEIGHT);
        connect(cThread[i], SIGNAL(imageAquired(const QImage,int  )),
                this, SLOT(displayImage( QImage,int)));
        cThread[i]->start();
    }
    this->areCamerasRunning=true;
}


void MainWindow::stopCameras()
{
    if (this->areCamerasRunning)
    {
        for (int i=0; i<8;i++)
        {
            disconnect(cThread[i], SIGNAL(imageAquired(const QImage,int  )),
                       this, SLOT(displayImage( QImage,int)));
            cThread[i]->stop();
            while(!cThread[i]->isFinished()) cThread[i]->wait();
            delete cThread[i];
        }
    }
    this->areCamerasRunning=false;
}

MainWindow::~MainWindow()
{
    stopCameras();

}



void MainWindow::runCalibrate()
{
//    for (int i=0;i<8;i++){
//        cThread[i]->isCalibrating=true;
//    }
    stopCameras();
    isCalibrating=true;
    mThread=new CalibrateThread(640,480);
    connect(mThread, SIGNAL(calibrationSuccess(const QImage,int  )),
            this, SLOT(onCalibrationSuccess( QImage,int)));
    mThread->start();

}

void MainWindow::on_cmdQuit_clicked()
{
    this->stopCameras();
    this->close();
}

void MainWindow::on_cmdCapture_clicked()
{
    if (isCalibrating){
        disconnect(mThread, SIGNAL(calibrationSuccess(const QImage,int  )),
                   this, SLOT(onCalibrationSuccess( QImage,int)));
        mThread->stop();
        mThread->wait();
    }
    runCameras();
}

void MainWindow::onCalibrationSuccess(QImage img, int device){

        lblCam[device]->setPixmap(QPixmap::fromImage(img));
}
