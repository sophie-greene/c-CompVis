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
    vector<string>filelist;
    //sleep(5);
    for(int i=0;i<8;i++){
        filelist.clear();
        cap.open(i);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
        if(cap.isOpened()){
            qDebug()<<"...................cam "<<1<<"openned";
            for (int j=0;j<20;j++){
                cap >>imgCam;
                string name=boost::lexical_cast<string>(j);
                string cam=boost::lexical_cast<string>(i);
                name="calImg_cam"+cam+"/calibrate"+name+".jpg";
                filelist.push_back(name);
                //qDebug()<<name.c_str();
                imwrite(name.c_str(),imgCam);


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
            //  sleep(1);
        }

        }
        else
        {
           // qDebug()<<"cam not open, video"<<i;
            this->stopped=true;

        }
        calibrate(filelist);
        cap.release();
    }
    //calibrate(i);
    this->stopped=true;

}
void CalibrateThread::calibrate(const vector<string>& filelist){
    Mat image;
    qDebug()<<filelist.size();
    for (int i=0;i<filelist.size();i++)
        image= imread(filelist[i].c_str(),0);
    // Create calibrator object
    CameraCalibrator cameraCalibrator;
    // add the corners from the chessboard
    Size boardSize(5,4);
    cameraCalibrator.addChessboardPoints(
            filelist,	// filenames of chessboard image
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
bool storeMartix(string filename,Mat cameraMatrix ,Mat distCoeffs)
{
    FileStorage fs(filename, FileStorage::WRITE);

    fs << "frameCount" << 5;
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    fs<< "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs<< "features" << "[";
    for( int i = 0; i < 3; i++ )
    {
        int x = rand() % 640;
        int y = rand() % 480;
        uchar lbp = rand() % 256;

        fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for( int j = 0; j < 8; j++ )
            fs << ((lbp >> j) & 1);
        fs << "]" << "}";
    }
    fs << "]";
    fs.release();
    return true;
}

//    string name=boost::lexical_cast<string>(device);
//    name="img/calibrate"+name+".bmp";
//    img.save(name.c_str(),"BMP");

