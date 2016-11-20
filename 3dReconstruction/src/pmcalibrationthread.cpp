#include "include/pmcalibrationthread.h"

PhotoMCalibrationThread::PhotoMCalibrationThread():QThread()
{
    stopped=false;
}
void PhotoMCalibrationThread::pmCalibrate(int device,const vector<string>& filelist )
{
    Mat image;
    //qDebug()<<filelist.size();
    image= imread(filelist[0].c_str(),0);
    // Create calibrator object
    CameraCalibrator pmCalibrator;
    // add the corners from the chessboard
    vector<string> successflst;
    Size boardSize(5,4);
    pmCalibrator.addChessboardPoints(
            filelist,	// filenames of chessboard image
            boardSize,successflst);	// size of chessboard

    Size imSize(image.size().width,image.size().height);
    vector<vector<Point2f> > ip=pmCalibrator.getImagePoints();
    vector<vector<Point3f> > op=pmCalibrator.getObjectPoints();
//    qDebug()<<"ip size"<<ip.size()<<"op size"<<op.size();
//    qDebug()<<"ip point"<<ip[0][0].x<<","<<ip[0][0].y;
//    qDebug()<<"op point"<<op[0][0].x<<","<<op[0][0].y<<","<<op[0][0].z;
    Mat imPoints=pmCalibrator.convertVector2fToMat(ip);
    Mat obPoints=pmCalibrator.convertVector3fToMat(op);
    QString s=QString::fromStdString("pmcalib_data");
    if(!QDir("Folder").exists(s))
        QDir().mkdir(s);
    string filename=boost::lexical_cast<string>(device);
    FileStorage fs("pmcalib_data/cam"+filename+"ObImPoints.yml", FileStorage::WRITE);

    fs<< "Object" << obPoints;
    fs<<"Image"<< imPoints;
    fs<<"FileList"<<successflst;
    fs.release();
}

void PhotoMCalibrationThread::run()
{
    //read calibration images
    vector<string> filelst;
    string sfile;

        for (int cam=0;cam<8;cam++){
            for (int light=0;light<8;light++)
            {
                for (int angle=0;angle<=90;angle=angle+30)
                {
                sfile=PM_IMAGES_PATH+boost::lexical_cast<string>(angle)+"/"+
                      "cam"+boost::lexical_cast<string>(cam)+
                      "Light"+boost::lexical_cast<string>(light)+".jpg";
                filelst.push_back(sfile.c_str());
                //qDebug()<<sfile.c_str();
            }

        }
 pmCalibrate(cam,filelst);
    }

    emit this->pmcalibrationSuccess(true);
}

void PhotoMCalibrationThread::stop(){
    this->stopped=true;
}

