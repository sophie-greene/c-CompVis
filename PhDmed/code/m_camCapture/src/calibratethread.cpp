#include "src/calibratethread.h"

CalibrateThread::CalibrateThread():QThread()
{
    stopped=false;
}

void CalibrateThread::run()
{
    qDebug()<<"Calibrate thread";
    Mat imgCam[8];
    //capture images
    for(int i=0;i<8;i++){
        imgCam[i]=capture(i,640,480,0,0,0,0);
    }
    //find chess board corners
    // output vectors of image points
    vector<Point2f> imageCorners;
    // number of corners on the chessboard

    Size boardSize(5,4);
    // Get the chessboard corners
    bool found = findChessboardCorners(imgCam[4],  boardSize, imageCorners);
    //Draw the corners
    drawChessboardCorners(imgCam[4], boardSize, imageCorners,found); // corners have been found
   imwrite("cal.bmp",imgCam[4]);
//   for (int j=0;j<8;j++)
//   {
//       string s=boost::lexical_cast<string>(j);
//       s="camera"+s;
//       namedWindow(s.c_str(),j);
//       imshow(s.c_str(), imgCam[j]);
//    }
    emit this->calibrationSuccess(true);
}

void CalibrateThread::stop(){
    stopped=true;
}
