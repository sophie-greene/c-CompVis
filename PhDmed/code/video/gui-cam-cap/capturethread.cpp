#include "capturethread.h"

CaptureThread::CaptureThread(int d,int w,int h) :
        QThread()
{
    this->device=d;
    this->width=w;
    this->height=h;
    this->stopped=false;
    this->isCalibrating=false;
    this->cap.open(this->device);
    if(cap.isOpened() )  // check if we succeeded
    {
        //change resolution and compression of v4l2
        cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
        cap.set(CV_CAP_PROP_FPS,10);
        this->deviceOpened=true;
    }
    else
    {
        this->deviceOpened=false;
        this->stopped=true;
    }
}

void CaptureThread::run()
{
    Mat imgCam;

    while(!this->stopped){
        if(this->stopped)break;
        if(this->cap.isOpened()){
            this->cap >>imgCam;

            cvtColor(imgCam, imgCam, CV_BGR2RGB);
            QImage qimg= QImage((const unsigned char*)(imgCam.data),
                                imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);
            emit this->imageAquired(qimg,this->device);
        }
        else
        {
            qDebug()<<"cam not open, video"<<this->device;
        }

    }

}

void CaptureThread::stop(){

    this->stopped=true;

}

int CaptureThread::getHeight()
{
    return this->height;
}

int CaptureThread::getWidth()
{
    return this->width;
}

