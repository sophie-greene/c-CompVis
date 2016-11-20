#include "include/laserdatathread.h"

LaserDataThread::LaserDataThread(int w,int h) :  QThread()
{
    this->width=w;
    this->height=h;
    this->stopped=false;
    this->threadCnt=0;
    for (int i=0; i<8;i++){
        cap[i].open(i);
     cap[i].set(CV_CAP_PROP_FRAME_WIDTH, 176);
     cap[i].set(CV_CAP_PROP_FRAME_HEIGHT, 144);
     cap[i].set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
     }
    //qDebug()<<"Laser Thread starting";
    FileStorage fs("point.yml", FileStorage::WRITE);
    fs.release();
}

void LaserDataThread::run()
{
    vector <vector <Point> > Points;
    vector <Point> imgPoints;
   // qDebug()<<"Laser Thread running";
    while(!this->stopped)
    {
        threadCnt=threadCnt+1;
        if (this->stopped)break;
        for (int i=0; i<8;i++)
        {

         //cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
            //cap.open(i);
            if(cap[i].isOpened())
            {
                 string name=boost::lexical_cast<string>(i);
                 name="cam"+name+"/frame"+ boost::lexical_cast<string>(threadCnt)+".bmp";
                Mat img,imgHSV,imgCam;
               // cap[i]
                  //  qDebug()<<"fps "<<cap[i].get(CV_CAP_PROP_FPS);
                // qDebug()<<"width "<<cap[i].get(CV_CAP_PROP_FRAME_WIDTH);
                // qDebug()<<"height "<<cap[i].get(CV_CAP_PROP_FRAME_HEIGHT);

                cap[i].read(img);
                imwrite(name,img);
                cvtColor(img, imgCam, CV_BGR2RGB);

               // imgSet.push_back(qimg);
                cvtColor(img, imgHSV, CV_RGB2GRAY);
                int x,y;
                detectPoint(imgHSV,x,y);
                circle(imgCam,Point(x,y),1,Scalar(0,255,0),1);


               QImage qimg= QImage((const unsigned char*)(imgCam.data),
                                    imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);
                emit this->LaserAquis(qimg,i);
                imgPoints.push_back(Point(x,y));
               qDebug()<<"Thread"<<threadCnt<<"device "<<i<<" " <<x<<","<<y;

            }
            //cap.release();
           // cap.release();
        }
       // writePoint(imgPoints);
        int cnt=0;
        for(int i=0;i<imgPoints.size();i++)
        {
            if (imgPoints[i].x>0 && imgPoints[i].y>0)cnt=cnt+1;
        }
        if(cnt>1)writePoint(imgPoints);//Points.push_back(imgPoints);
       imgPoints.erase(imgPoints.begin(),imgPoints.end());
       // qDebug()<<imgPoints.size();
   // qDebug()<<"**************************************************capture ends";
       // cvWaitKey();
    }
    for(int i=0;i<8;i++){
        cap[i].release();
    }
qDebug()<<Points.size();
//FileStorage fs("point.yml", FileStorage::WRITE);
//   // for(int i=0;i<Points.size();i++){
//        fs <<"points"<< Mat(Points);
//   // }
//    fs.release();
   // sleep(10);
    qDebug()<<"finished writing";

}
void LaserDataThread::writePoint(const vector<Point> & pts)
{
    FileStorage fs("point.yml", FileStorage::APPEND);
       // for(int i=0;i<pts.size();i++){
            fs <<"points"<< pts;
        //}
        fs.release();
}
void LaserDataThread::stop()
{
    this->stopped=true;
}

int LaserDataThread::getHeight()
{
    return this->height;
}

int LaserDataThread::getWidth()
{
    return this->width;
}
void LaserDataThread::detectPoint(const Mat & img,int & x,int & y)
{
    Point pt;
    vector <Mat> spImg;
    double minVal;
    double maxVal;
    Point minIdx;
    Point maxIdx;

    minMaxLoc(img, &minVal, &maxVal,&minIdx, &maxIdx );
   // qDebug()<<"max val "<<maxVal;
   // qDebug()<<"index"<<maxIdx.x<<" "<<maxIdx.y;
    if (maxVal>70){
        x=maxIdx.x;
        y=maxIdx.y;
    }else{
        x=-1;
        y=-1;
    }

}
