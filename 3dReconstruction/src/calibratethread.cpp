#include "include/calibratethread.h"

CalibrateThread::CalibrateThread(int w, int h, int frames):QThread()
{

    this->width=w;
    this->height=h;
    this->noOfFarmes=frames;
    stopped=false;

}

void CalibrateThread::run()
{
    Mat imgCam;

    vector<string> filelist[8];
    vector<vector<vector<Point2f> > > imgPoints;
    //sleep();
    for (int j=0;j<this->noOfFarmes;j++){

        for (int device=0;device<8;device++){
            string cam="calImg_cam"+boost::lexical_cast<string>(device);
            // cap.open(device);
            // if(cap.isOpened()){
            //   cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
            //   cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
            //    cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
            string name=boost::lexical_cast<string>(j);
            QString s=QString::fromStdString(cam);
            if(!QDir("Folder").exists(s))
                QDir().mkdir(s);
            name=cam+"/calibrate"+name+".bmp";
            filelist[device].push_back(name);
            //imgCam=capture (device,width,height,BRIGHTNESS,CONTRAST,SATURATION,GAIN );
            //for(int k=0;k<2;k++) cap>>imgCam;
            Mat imgCam1=imgCam;
            imwrite(name.c_str(),imgCam);

            //resize(imgCam, imgCam, Size(640,480), 0, 0, CV_INTER_CUBIC);
            //qDebug()<<name.c_str();

            // qDebug()<<"...................cam "<<device<<"openned";
            // output vectors of image points
            vector<Point2f> imageCorners;
            // number of corners on the chessboard
            Size boardSize(5,4);
            // Get the chessboard corners
            bool found =findChessboardCorners(imgCam1,boardSize, imageCorners);

            //Draw the corners
            drawChessboardCorners(imgCam1,boardSize, imageCorners,found); // corners have been found

            cvtColor(imgCam1, imgCam1, CV_BGR2RGB);

            QImage qimg= QImage((const unsigned char*)(imgCam1.data),
                                imgCam1.cols,imgCam1.rows,imgCam1.step,QImage::Format_RGB888);
            emit this->calibrationSuccess(qimg,device);
            // cap.release();

            // }
        }

    }


    for (int device=0;device<8;device++){
        //for (int i=0;i<filelist[device].size();i++)
        //        qDebug()<<filelist[device].at(i).c_str();
        vector<vector <Point2f> >temp;

        calibrate(device,filelist[device],temp);
        imgPoints.push_back(temp);
    }
    vector <vector <Point2f> > pts;
    vector <Point2f>  aux;
    for(int i=0;i<imgPoints.size();i++){
       // aux.push_back(imgPoints[i]);
        vector<vector <Point2f> > temp=imgPoints[i];
        for(int j=0; j<temp.size();j++){
            pts.push_back( temp[j]);
     }


    }
    string l;
    Mat fundamental_matrix;
    for(int i=0;i<8;i++)
    {
        for (int j=0;j<8;j++)
        {

            if(i==j)continue;
            fundamental_matrix=
                    findFundamentalMat(pts[i], pts[j], FM_RANSAC, 3, 0.99);
            l="fundMat/cam"+boost::lexical_cast<string>(i)+"cam"+boost::lexical_cast<string>(j)+".yml";

            QString s=QString::fromStdString("fundMat");
            if(!QDir("Folder").exists(s))
                QDir().mkdir(s);


            //qDebug()<<"fineame >>>>>>>>>>>>>>"<<l.c_str();
            FileStorage fs1(l.c_str(), FileStorage::WRITE);
            fs1<<"fundamental"<<fundamental_matrix;
            fs1.release();
            //qDebug()<<"fundementalmatrix>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<fundamental_matrix.at<double>(0,0);
        }
    }

    this->stopped=true;
    //qDebug()<<">>>>>>>>>>>>>>>calibration ended";

}
void CalibrateThread::calibrate(int device,const vector<string>& filelist,vector<vector<Point2f> > & imgPoints ){
    // qDebug()<<filelist[1].c_str();
    Mat image;
    image= imread(filelist[0].c_str(),0);
    // Create calibrator object
    CameraCalibrator cameraCalibrator;
    // add the corners from the chessboard
    vector<string>  successflst;
    Size boardSize(5,4);
    cameraCalibrator.addChessboardPoints(
            filelist,	// filenames of chessboard image
            boardSize,successflst);	// size of chessboard

    // calibrate the camera
    cameraCalibrator.setCalibrationFlag(true,true);
    Size imSize(image.size().width,image.size().height);
    string filename=boost::lexical_cast<string>(device);
    cameraCalibrator.calibrateI(imSize);
    //cameraCalibrator.calibrateE(imSize);
    //cameraCalibrator.stereoCal()
    Mat im=cameraCalibrator.remap(image);
    //qDebug()<<im.cols<<","<<im.rows;
    imwrite(filename+"undis.jpg",im);
    //store camera calibration matrices
    //get extrinsic (R+T)

    Mat cameraMatrix,distCoeffs,rotationMatrix,translationVector;
    QString s=QString::fromStdString("calib_data");
    if(!QDir("Folder").exists(s))
        QDir().mkdir(s);
    FileStorage fs("calib_data/cam"+filename+"Intrinsics.yml", FileStorage::WRITE);
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));

    cameraMatrix=cameraCalibrator.getCameraMatrix();
    fs<< "cameraMatrix" << cameraMatrix;

    distCoeffs=cameraCalibrator.getDistCoeffs();
    fs<<"DistCoeffs"<< distCoeffs;

    fs.release();
    fs.open("calib_data/cam"+filename+"Extrinsic.yml", FileStorage::WRITE);
    fs << "calibrationDate" << asctime(localtime(&rawtime));

    translationVector=cameraCalibrator.getTranslationVector();
    fs<<"translationVector"<< translationVector;
    rotationMatrix=cameraCalibrator.getRotationMatrix();
    fs<<"rotationMatrix"<<rotationMatrix;
    fs.release();
    vector<vector<Point3f> > obj=cameraCalibrator.getObjectPoints();
    vector <vector<Point2f> > temp=cameraCalibrator.getImagePoints();
    for (int i=0;i<temp.size();i++)
        imgPoints.push_back(temp[i]);

    QString sss=QString::fromStdString("point");
    if(!QDir("Folder").exists(sss))
        QDir().mkdir(sss);

    string l="point/cam"+filename+".yml";

    FileStorage fs1(l.c_str(), FileStorage::WRITE);
    fs1<<"Object"<<obj;
    fs1<<"Image"<<imgPoints;
    fs1.release();
    double apertureWidth=10,apertureHeight=5;
    double fovx,fovy,focalLength;
    Point2d principalPoint;
    double aspectRatio;
    calibrationMatrixValues(cameraMatrix,imSize,
                            apertureWidth,
                            apertureHeight,
                            fovx,
                            fovy,
                            focalLength,
                            principalPoint,
                            aspectRatio);
    qDebug()<<"cam"<<device<<" fovx "<<fovx<<" fovy "<< fovy<<" focalLength(mm) "<<focalLength;

//    if(!cameraCalibrator.getRotationMatrix().empty()){
//        cameraMatrix=cameraCalibrator.getRotationMatrix();
////        qDebug() << " Camera intrinsic: " <<
////                cameraMatrix.rows << "x" <<
////                cameraMatrix.cols << endl;
////        qDebug()<< cameraMatrix.at<double>(0,0) << " "
////                << cameraMatrix.at<double>(0,1) << " "
////                << cameraMatrix.at<double>(0,2) << endl;
////        qDebug() << cameraMatrix.at<double>(1,0) << " "
////                << cameraMatrix.at<double>(1,1)  << " "
////                << cameraMatrix.at<double>(1,2) << endl;
////        qDebug() << cameraMatrix.at<double>(2,0) << " "
////                << cameraMatrix.at<double>(2,1) << " "
////                << cameraMatrix.at<double>(2,2) << endl;
//    }
}
void CalibrateThread::stereoCalib(int device1, const vector<string>& filelist1,int device2, const vector<string>& filelist2){

}

void CalibrateThread::stop(){
    stopped=true;
}


//    string name=boost::lexical_cast<string>(device);
//    name="img/calibrate"+name+".bmp";
//    img.save(name.c_str(),"BMP");

