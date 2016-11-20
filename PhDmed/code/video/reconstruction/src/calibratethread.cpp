#include "calibratethread.h"

CalibrateThread::CalibrateThread(int w, int h):QThread()
{
    this->width=w;
    this->height=h;
    stopped=false;
}

void CalibrateThread::run()
{

    Mat imgCam;
    VideoCapture cap;
    //sleep(5);
    for(int i=0;i<8;i++){
        cap.open(i);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
        if(cap.isOpened()){
            qDebug()<<"...................cam "<<i<<"openned";
            for (int j=0;j<2;j++){
                cap >>imgCam;
                string name=boost::lexical_cast<string>(i);
                string cam=boost::lexical_cast<string>(j);
                name="calImg/calibrate_"+cam+"_"+name+".jpg";
                fileList.push_back(name);
                //qDebug()<<name.c_str();
                imwrite(name.c_str(),imgCam);
                // sleep(1);
            }
            // output vectors of image points
            vector<Point2f> imageCorners;
            // number of corners on the chessboard
            Size boardSize(5,4);
            // Get the chessboard corners
            bool found =findChessboardCorners(imgCam,boardSize, imageCorners);

            //Draw the corners
            drawChessboardCorners(imgCam,boardSize, imageCorners,found); // corners have been found
            cvtColor(imgCam, imgCam, CV_BGR2RGB);
            QImage qimg= QImage((const unsigned char*)(imgCam.data),
                                imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);

            emit this->calibrationSuccess(qimg,i);

        }
        else
        {
            qDebug()<<"cam not open, video"<<i;
        }
        cap.release();
    }
    calibrate();
    this->stopped=true;

}
void CalibrateThread::calibrate(){
    Mat image;
qDebug()<<fileList.size();
    for (int i=0;i<fileList.size();i++)
        image= imread(fileList[i].c_str(),0);



    // Create calibrator object
    CameraCalibrator cameraCalibrator;
    // add the corners from the chessboard
    Size boardSize(5,4);
    cameraCalibrator.addChessboardPoints(
            fileList,	// filenames of chessboard image
            boardSize);	// size of chessboard
    // calibrate the camera
    //	cameraCalibrator.setCalibrationFlag(true,true);
    Size imSize(image.size().width,image.size().height);

    cameraCalibrator.calibrate(imSize);
    // display camera matrix
    Mat cameraMatrix= cameraCalibrator.getCameraMatrix();
    qDebug() << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
    qDebug()<< cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << endl;
    qDebug() << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << endl;
    qDebug() << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << endl;

}

void CalibrateThread::stop(){
    stopped=true;
}


//    string name=boost::lexical_cast<string>(device);
//    name="img/calibrate"+name+".bmp";
//    img.save(name.c_str(),"BMP");

