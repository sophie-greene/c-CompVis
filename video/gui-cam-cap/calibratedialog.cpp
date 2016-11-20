#include "calibratedialog.h"


CalibrateDialog::CalibrateDialog(QWidget *parent) :
        QDialog(parent)
{
    cmdQuit=new QPushButton(tr("&Quit"));
    connect(cmdQuit ,
            SIGNAL(clicked()),
            this,
            SLOT(on_cmdQuit_clicked()));
    widgetResize();
    runCalibration();
}
void CalibrateDialog::widgetResize()
{
    //get screen resolution
    int w=80*(QApplication::desktop()->availableGeometry().size().width())/100;
    int h=95*(QApplication::desktop()->availableGeometry().size().height())/100;
    int left=20*(QApplication::desktop()->availableGeometry().size().width())/100;
    int top=5*(QApplication::desktop()->availableGeometry().size().height())/100;
    int camFrameWidth=(w/3)-(w/20);
    int camFrameHeight=(h/3)-(h/20);

    int leftRightMargins=(w-(camFrameWidth*3))/4;
    int topBottomMargins=(h-(camFrameHeight*3))/4;

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setContentsMargins(leftRightMargins,
                                   topBottomMargins,
                                   leftRightMargins,
                                   topBottomMargins);
   //lblMsg=new QLabel(tr("Calibrating ............"));
   //lblMsg->setFixedHeight(30);
   //mainLayout->addWidget(lblMsg,0,0);

    int row=1;
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
    cmdQuit->setFixedWidth(camFrameWidth/3);

    mainLayout->addWidget(cmdQuit,6,2,1,1,Qt::AlignCenter);


    setLayout(mainLayout);

    setWindowTitle(tr("Calibration"));
}

void CalibrateDialog::runCalibration()
{
        cThread=new CalibrateThread(640,480);
        connect(cThread, SIGNAL(calibrationSuccess(const QImage,int  )),
                this, SLOT(onCalibrationSuccess( QImage,int)));
        cThread->start();
}

CalibrateDialog::~CalibrateDialog()
{

}



void CalibrateDialog::on_cmdQuit_clicked()
{
    destroyThread();
    this->close();
}
void CalibrateDialog::destroyThread(){

        disconnect(cThread, SIGNAL(calibrationSuccess(const QImage,int  )),
                   this, SLOT(onCalibrationSuccess( QImage,int)));
        cThread->stop();
        cThread->wait();

}

void CalibrateDialog::onCalibrationSuccess(QImage img, int device){
//if(device==7)
       //lblMsg->setText("Calibration performed successfully");


        lblCam[device]->setPixmap(QPixmap::fromImage(img));

}
