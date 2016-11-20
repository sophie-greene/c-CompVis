#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include <QDebug>
#include <boost/lexical_cast.hpp>
#include<capturethread.h>
using namespace cv;
using namespace std;

int main()
{
    CaptureThread * cThread;
    cThread=new CaptureThread(0,160,120);
    connect(cThread, SIGNAL(imageAquired(const QImage &)),
              this, SLOT(displayImage(const QImage &)));
    cThread->start();
return 0 ;
}
displayImage(const QImage & img){
    img.save("qimg.bmp","BMP");
}
