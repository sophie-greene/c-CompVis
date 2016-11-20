#include "include/mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent)

{
    //reset pint.yml
    FileStorage fs("point.yml", FileStorage::WRITE);
    fs.release();
    creatMenu();
    cmdLightOn=new QPushButton (tr("On"));
    cmdLightOff=new QPushButton (tr("Off"));
    this->cmbLights=new QComboBox();
    for (int i=0;i<8;i++)
    {
        string s="Light "+boost::lexical_cast<std::string>(i)+"  ";
        cmbLights->addItem(s.c_str(),QVariant(i));
    }


    connect(cmdLightOn ,
            SIGNAL(clicked()),
            this,
            SLOT(on_cmdLightOn_clicked()));
    connect(cmdLightOff ,
            SIGNAL(clicked()),
            this,
            SLOT(on_cmdLightOff_clicked()));

    widgetResize();
    areCamerasRunning=false;
    isCalibrating=false;
    isLaser=false;

}
void MainWindow::creatMenu()
{
    menu=new QMenu("&File");
    menuBar()->addMenu(menu);

    startCamAction=new QAction("Start Cameras",this);
    menu->addAction(startCamAction);

    gCalibrateAction=new QAction("Geometric Calibration",this);
    menu->addAction(gCalibrateAction);

    intrCalibrateAction=new QAction("Intrinsic Calibration",this);
    menu->addAction(intrCalibrateAction);

    pmCalibrateAction=new QAction("Photometric Calibration",this);;
    menu->addAction(pmCalibrateAction);

    getLaser=new QAction("Laser Data Aquisition",this);;
    menu->addAction(getLaser);

    quitAction=new QAction("Exit",this);;
    menu->addAction(quitAction);
    connect(startCamAction ,
            SIGNAL(triggered()),
            this,
            SLOT(on_menuStartCam()));

    connect(gCalibrateAction ,
            SIGNAL(triggered()),
            this,
            SLOT(on_menuCalib()));

    connect(intrCalibrateAction ,
            SIGNAL(triggered()),
            this,
            SLOT(on_menuIntrCalib()));

    connect(pmCalibrateAction ,
            SIGNAL(triggered()),
            this,
            SLOT(on_menuPMCalib()));

    connect(getLaser ,
            SIGNAL(triggered()),
            this,
            SLOT(on_menuLaser()));

    connect(quitAction ,
            SIGNAL(triggered()),
            this,
            SLOT(on_menuQuit()));
}
//size and layout
void MainWindow::widgetResize()
{
    //get screen resolution
    int w=70*(QApplication::desktop()->availableGeometry().size().width())/100;
    int h=70*(QApplication::desktop()->availableGeometry().size().height())/100;
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
    menuBar()->setGeometry(QRect(30, 30, camFrameWidth, 50));
    //mainLayout->setMenuBar(menuBar);
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
        //qDebug()<<"i="<<i<<"("<<row<<","<<col<<")";

    }
    QGridLayout* light=new QGridLayout();
    QLabel* lbl=new QLabel(tr("Choose Light"));
    light->addWidget(lbl,0,0,2,2);
    light->addWidget(cmbLights,2,0,2,2);
    light->addWidget(cmdLightOn,4,0);
    light->addWidget(cmdLightOff,4,1);
    light->setMargin(80);
    light->setVerticalSpacing(0);

    light->geometry().setWidth(camFrameWidth/3);
    //lbl->setFixedWidth(camFrameWidth/2);
    //cmbLights->setFixedWidth(camFrameWidth);
    //cmdLightOn->setFixedWidth(camFrameWidth/4);
    //cmdLightOff->setFixedWidth(camFrameWidth/4);
    mainLayout->addLayout(light,5,2,1,1,Qt::AlignBottom|Qt::AlignRight);
    QWidget* widget = new QWidget(this);
    widget->setLayout(mainLayout);
    this->setCentralWidget(widget);
    //setLayout(mainLayout);


    setWindowTitle(tr("3D Reconstruction "));
}
void MainWindow::displayImage( QImage  img,int device,int x,int y)
{
    //    QPixmap pixmap=QPixmap::fromImage(img);
    //    QPainter pixPaint(&pixmap);
    //    QBrush brush2( Qt::green);
    //    pixPaint.setBrush(brush2);
    //    pixPaint.drawEllipse(QPoint(x,y),2,2);
    lblCam[device]->setPixmap(QPixmap::fromImage(img));
}
void MainWindow::onlaserAquired(int device,int thread,int x,int y)
{
    vector<Point> framePoint(8,Point(-2,-2));
    Point tmp;
    tmp.x=x;
    tmp.y=y;
    if (points.size()>=thread)
    {

        framePoint=points[thread-1];
        framePoint[device]=tmp;
        points[thread-1]=framePoint;
    }else
    {
        framePoint[device]=tmp;
        points.push_back(framePoint);
    }
    if (allFrames(framePoint)){
        writePoint(points[thread-1]);
        //points.erase(thread-1);
    }
   // qDebug()<<points.size()<<" device ******"<<device<<"Thread no. "<<thread<<" Point"<<x<<" "<<y;

}
bool MainWindow::allFrames(const vector<Point> & pt){
    bool isfull=false;
    int cnt=0;
    for (int i=0;i<8;i++){
        if (pt[i].x!=-2 || pt[i].x!=-2 )cnt=cnt+1;
        if(pt[i].x==-2 || pt[i].x==-2 ){
            isfull=false;
            break;
        }
    }
    if (cnt==8) return true;
    else return false;
}

void MainWindow::writePoint(const vector<Point> & pts)
{

    int cnt=0;
    for(int i=0;i<pts.size();i++)
    {
        if (pts[i].x>0 && pts[i].y>0)cnt=cnt+1;
    }

    if (cnt>3)
    {
        FileStorage fs("point.yml", FileStorage::APPEND);
        fs <<"points"<< pts;
        fs.release();
    }
}

void MainWindow::runCameras()
{
    for (int i=0; i<8;i++)
    {
        cThread[i]=new CaptureThread(i,FR_WIDTH,FR_HEIGHT);
        connect(cThread[i], SIGNAL(imageAquired(const QImage,int,int,int)),
                this, SLOT(displayImage( QImage,int,int,int)));
        connect(cThread[i], SIGNAL(laserAquired(int,int,int,int)),
                this, SLOT(onlaserAquired(int,int,int,int)));
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
            disconnect(cThread[i], SIGNAL(imageAquired(const QImage,int,int,int )),
                       this, SLOT(displayImage( QImage,int,int,int)));
            disconnect(cThread[i], SIGNAL(laserAquired(int,int,int,int)),
                       this, SLOT(onlaserAquired(int,int,int,int)));
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
    if(isLaser){

        disconnect(lThread, SIGNAL(LaserAquis(const  QImage, int)),
                   this, SLOT(onLaserAquis( QImage, int)));
        lThread->stop();
        lThread->wait();
       // delete lThread;
    }
}



void MainWindow::runCalibrate()
{
    //    for (int i=0;i<8;i++){
    //        cThread[i]->isCalibrating=true;
    //    }
    stopCameras();
    isCalibrating=true;

    mThread=new CalibrateThread(640,480,1);

    connect(mThread, SIGNAL(calibrationSuccess(const QImage,int  )),
            this, SLOT(onCalibrationSuccess(QImage,int)));
    mThread->start();

}

void MainWindow::on_menuQuit()
{
    this->stopCameras();

    if(isLaser){

        disconnect(lThread, SIGNAL(LaserAquis(const  QImage, int)),
                   this, SLOT(onLaserAquis( QImage, int)));
        lThread->stop();
        lThread->wait();
        delete lThread;
    }
    this->close();
}
void MainWindow::on_menuCalib()
{
    this->runCalibrate();

}

void MainWindow::on_menuPMCalib()
{
    this->pmThread=new PhotoMCalibrationThread();
    connect(pmThread, SIGNAL(pmcalibrationSuccess(bool )),
            this, SLOT(onpmCalibrationSuccess(bool)));
    pmThread->start();
}

void MainWindow::on_cmdLightOn_clicked()
{
    //qDebug()<<"light on";
    this->lightControl(cmbLights->currentIndex(),true);
    cmbLights->setEnabled(false);
    cmdLightOn->setEnabled(false);

}
void MainWindow::on_cmdLightOff_clicked()
{
    //qDebug()<<"light off";
    //qDebug()<< cmbLights->currentIndex();
    this->lightControl(cmbLights->currentIndex(),false);
    cmbLights->setEnabled(true);
    cmdLightOn->setEnabled(true);

}

void MainWindow::onCalibrationSuccess(QImage img, int device){

    lblCam[device]->setPixmap(QPixmap::fromImage(img));
}
void MainWindow::lightControl(int light,bool status)
{
    //serial object for /dev/ttyACM0
    QextSerialPort * port = new QextSerialPort(QLatin1String(SERIAL), QextSerialPort::Polling);

    //write data to port
    std::string s;
    s= boost::lexical_cast<std::string>(light+2);
    //open port in write mode
    port->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if (status)port->write("1");
    else port->write("0");
    port->write(s.c_str());

    port->close();


}
void MainWindow::on_menuStartCam()
{
    if(this->startCamAction->text()=="Start Cameras"){
        this->startCamAction->setText("Stop Cameras");
        //        if (isCalibrating){
        //            disconnect(mThread, SIGNAL(calibrationSuccess(bool,int  )),
        //                       this, SLOT(onCalibrationSuccess( bool,int)));
        //            mThread->stop();
        //            mThread->wait();
        //        }
        runCameras();
    }else{
        this->startCamAction->setText("Start Cameras");
        for (int i=0; i<8;i++)
        {
            this->lblCam[i]->setPixmap(NULL);
            this->updateGeometry();
        }
        this->stopCameras();
    }

}

void MainWindow::onpmCalibrationSuccess(bool success){
    qDebug()<<"photometric calibration successfull";
}
void MainWindow::on_menuIntrCalib()
{
    stopCameras();
    isCalibrating=true;

    iThread[0]=new IntrCalibThread(0,640,480,false);//using intrinisics provided

    connect(iThread[0], SIGNAL(intrCalibrationSuccess(const QImage,int)),
            this, SLOT(onIntrCalibrationSuccess(QImage,int)));
    connect(iThread[0], SIGNAL(intrCalibStop(int,bool)),
            this, SLOT(onIntrCalibStopped(int,bool)));

    iThread[0]->start();
}
void MainWindow::onIntrCalibrationSuccess(QImage img, int device)
{
    if(!img.isNull())
        lblCam[device]->setPixmap(QPixmap::fromImage(img));

}
void MainWindow::onIntrCalibStopped(int device,bool stopped)
{
    if (stopped)
    {
        disconnect(iThread[device], SIGNAL(intrCalibrationSuccess(const QImage,int)),
                   this, SLOT(onIntrCalibrationSuccess(QImage,int)));
        disconnect(iThread[0], SIGNAL(intrCalibStop(int,bool)),
                   this, SLOT(onIntrCalibStopped(int,bool)));
        iThread[device]->stop();

        if(device<7)
        {
            iThread[device+1]=new IntrCalibThread(device+1,640,480,false);

            connect(iThread[device+1], SIGNAL(intrCalibrationSuccess(const QImage,int )),
                    this, SLOT(onIntrCalibrationSuccess(QImage,int)));
            connect(iThread[device+1], SIGNAL(intrCalibStop(int,bool)),
                    this, SLOT(onIntrCalibStopped(int,bool)));
            iThread[device+1]->start();
        }
    }
}
void MainWindow::on_menuLaser()
{
    // stopCameras();
    isLaser=true;
    lThread=new LaserDataThread(176,144);

    connect(lThread, SIGNAL(LaserAquis(const  QImage, int)),
            this, SLOT(onLaserAquis( QImage, int)));
    lThread->start();
}
void MainWindow::onLaserAquis(const QImage img, int dev)
{

    lblCam[dev]->setPixmap(QPixmap::fromImage(img));

    // qDebug()<<"Thread no."<<thread;
}
