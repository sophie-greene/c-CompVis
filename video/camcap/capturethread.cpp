
#include "capturethread.h"

CaptureThread::CaptureThread(int d,int w,int h) :
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
          string name=boost::lexical_cast<string>(this->device);
           name="camera"+name;
           namedWindow(name.c_str(),this->device);
           this->deviceOpen=true;
       }
    else
    {
        this->deviceOpen=false;
    }
}

void CaptureThread::run()
{
    Mat imgCam;

    while(!stopped){

    this->cap >>imgCam;
//    string name=boost::lexical_cast<string>(this->device);
//                name="camera"+name;
//                imshow(name, imgCam);
//    cvtColor(imgCam, imgCam, CV_BGR2RGB);
//
//    this->qimg= QImage((const unsigned char*)(imgCam.data),
//                       imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);

    emit this->imageAquired(putImage(imgCam));
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
QImage CaptureThread::getQimg()
{
    return this->qimg;
}
QImage CaptureThread::putImage(const Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(mat.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}
