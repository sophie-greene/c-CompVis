#include <QCoreApplication>
#include<iostream>
#include<core/core.hpp>
#include<highgui/highgui.hpp>
#include<cv.h>
#include <opencv.hpp>
//#include "video.hpp"
#include"uvc_camera.hpp"
#include<QDebug>
using namespace std;
using namespace cv;
//using namespace s9;
//using namespace s9::gl;
//using namespace s9::gl::compvis;
int main()
{
    //VidCam s("/dev/video0",640,480,15) ;
  //  s.getBuffer();
   UVCVideo s;
   boost::thread *pWorkerThread =  new boost::thread(s.startCapture("/dev/video0",640,480,15));


   qDebug()<<"buf"<<"hello";
    while(1){

      //cout<< s.pWorkerThread->attributes.get_stack_size();
        //VideoCapture cap(0);
        //Mat image;
        //cap>>image;
        unsigned char* buf=s.getBuffer();
      // qDebug()<< s.pWorkerThread->get_id();
        qDebug()<<"buf"<<buf;
        if (buf!=NULL){
            Mat imgbuf(Size(640, 480), CV_8UC3, buf);
            cv::cvtColor(imgbuf,imgbuf,CV_BGR2RGB);
            //   Mat image = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
            namedWindow("f2", CV_WINDOW_AUTOSIZE);
            imshow("f2",imgbuf);

        }
        if (waitKey())break;
    }
    return 0;

}
