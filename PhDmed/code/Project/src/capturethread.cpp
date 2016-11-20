#include "src/capturethread.h"
#include "src/uvccapture.h"
CaptureThread::CaptureThread(int w,int h) :
        QThread()
{

    this->width=w;
    this->height=h;
    stopped = false;
}

void CaptureThread::run()
{
Mat imgCam;

   // while (!stopped){
//
        QMutex mutex;
        mutex.lock();
      //  if (this->stopped)break;

        for(int i=0;i<8;i++){
           imgCam=capture(i,this->width,this->height,128,32,32,64);
       char* s;
          sprintf(s,"img%d.bmp",i);
qDebug()<< s;

           imwrite(s,imgCam);
            cvtColor(imgCam, imgCam, CV_BGR2RGB);

            this->qimg[i]= QImage((const unsigned char*)(imgCam.data),
                            imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);
            sprintf(s,"Qimg%d.bmp",i);
            qimg[i].save(s,"BMP");
           //qDebug()<< qimg[i]<<"......."<<i;

        }
        emit this->imageAquired(true);
        mutex.unlock();

   // }
}

void CaptureThread::stop()
{
    stopped = true;
}



int CaptureThread::getHeight()
{
    return this->height;
}

int CaptureThread::getWidth()
{
    return this->width;
}
QImage CaptureThread::getQimg(int index)
{
    return this->qimg[index];
}
