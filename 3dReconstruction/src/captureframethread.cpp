#include "include/captureframethread.h"

CaptureFrameThread::CaptureFrameThread(int d,int w,int h) :
        QThread()
{
    this->device=d;
    this->width=w;
    this->height=h;
    this->stopped=false;
    this->cap.open(this->device);
    if(cap.isOpened() )  // check if we succeeded
    {
        //change resolution and compression of v4l2
        cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
//        cap.set(CV_CAP_PROP_SATURATION,0.5);
//        cap.set(CV_CAP_PROP_GAIN,0.5);
//        cap.set(CV_CAP_PROP_CONTRAST,.5);
//       cap.set(CV_CAP_PROP_BRIGHTNESS,.64);
//       cap.set(CV_CAP_PROP_SHARPNESS,0.5);
        //cap.set(CV_CAP_PROP_WHITE_BALANCE,3724);

        this->deviceOpened=true;
    }
    else
    {
        this->deviceOpened=false;
        this->stopped=true;
    }
}

void CaptureFrameThread::run()
{
    Mat imgCam;
  int w=  cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int h=  cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  qDebug()<<w<<"x"<<h;
//    while(!this->stopped){
//        if(this->stopped)break;//thread ending while loop is running
        if(this->cap.isOpened()){
            this->cap >>imgCam;
//            cvtColor(imgCam, imgCam, CV_BGR2RGB);
//            QImage qimg= QImage((const unsigned char*)(imgCam.data),
//                                imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);
            emit this->imageAquired(imgCam,this->device);
        }
        else
        {
            qDebug()<<"cam not open, video"<<this->device;
        }

    //}

}

void CaptureFrameThread::stop(){

    this->stopped=true;

}

int CaptureFrameThread::getHeight()
{
    return this->height;
}

int CaptureFrameThread::getWidth()
{
    return this->width;
}
