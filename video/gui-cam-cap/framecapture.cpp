#include "framecapture.h"

FrameCaptureThread::FrameCaptureThread(int d,int w,int h) :
        QThread()
{
    this->device=d;
    this->width=w;
    this->height=h;
    stopped=false;
    this->cap.open(this->device);
    if(cap.isOpened() )  // check if we succeeded
    {

        cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );

        this->deviceOpen=true;
    }
    else
    {
        this->deviceOpen=false;
    }
}

void FrameCaptureThread::run()
{
    Mat imgCam;


        if(this->cap.isOpened()){
            this->cap >>imgCam;
            qDebug()<<"...................cam open, thread of cam"<<this->device;
                string name=boost::lexical_cast<string>(this->device);
                            name="camera"+name;
                           imshow(name, imgCam);
            cvtColor(imgCam, imgCam, CV_BGR2RGB);

            QImage qimg= QImage((const unsigned char*)(imgCam.data),
                                imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);

            qimg.save(name.c_str(),"BMP");

        }
        else
        {
            qDebug()<<"cam not open, video"<<this->device;
        }


}

void FrameCaptureThread::stop(){
    this->stopped=true;

}
void FrameCaptureThread::terminate(){
    this->stopped=true;
    this->cap.release();
    this->dumpObjectTree();
    cap=NULL;
   this->exit();
   delete this;
}


int FrameCaptureThread::getHeight()
{
    return this->height;
}

int FrameCaptureThread::getWidth()
{
    return this->width;
}

