#include "include/intrcalibthread.h"

IntrCalibThread::IntrCalibThread(int device,int width, int height,bool useIntrinsics):QThread()
{
    this->device=device;
    this->width=width;
    this->height=height;
    this->useIntrinsics=useIntrinsics;
    stopped=false;
    this->mustInitUndistort=true;
    //this->readCamPar();
    //int sz[3] = {2,2,2};
//initialise camera matrix
   cameraMatrix = (Mat_<double>(3,3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);

}
void IntrCalibThread::run()
{
    Mat img,imgboard;
    vector<Mat> rvecs, tvecs;
    Mat rvec,tvec;

    int cnt=0;

    string folder="/usr/not-backed-up/Dropbox/CalibData/video"+boost::lexical_cast<string>(this->device);
    if(!useIntrinsics){
        string file=folder+"/frame"+boost::lexical_cast<string>(cnt)+".bmp";
        while(access(file.c_str(),0)==0)
        {

            qDebug()<<file.c_str();

            img=imread(file.c_str());
            imgboard=img;
            Size boardSize(5,4);
            addChessboardPoints(img, boardSize,imgboard) ;
            if(!imgboard.empty())
            {
//                computeWorld();
//                line( imgboard, world[device][0], world[device][1], Scalar(0, 255, 0), 2);
//                line( imgboard, world[device][0], world[device][2], Scalar(255, 0, 0), 2 );
//                line( imgboard, world[device][0], world[device][3], Scalar(0, 0, 255), 2 );
                cvtColor(imgboard, imgboard, CV_BGR2RGB);
                QImage qimg= QImage((const unsigned char*)(imgboard.data),
                                    imgboard.cols,imgboard.rows,imgboard.step,QImage::Format_RGB888);
                emit this->intrCalibrationSuccess(qimg,device);

            }
            cnt=cnt+20;
            file=folder+"/frame"+boost::lexical_cast<string>(cnt)+".bmp";
        }
        if(objectPoints.size()>0 &&imagePoints.size()>0)
        {

            //intrinsics
            double temp=
                    calibrateCamera(objectPoints, // the 3D points
                                    imagePoints,  // the image points
                                    Size(width,height),    // image size
                                    cameraMatrix, // output camera matrix
                                    distCoeffs,   // output distortion matrix
                                    rvecs, tvecs, // Rs, Ts
                                    CV_CALIB_ZERO_TANGENT_DIST+CV_CALIB_RATIONAL_MODEL);        // set options



            qDebug()<<temp;
            Mat cam,dist;

            cameraMatrix.copyTo(cam);
            distCoeffs.copyTo(dist);
      this->displayMatrix(cam);
            //objectPoints.erase(objectPoints.begin()+1,objectPoints.end());
           // imagePoints.erase(imagePoints.begin()+1,imagePoints.end());
            //rvecs.erase(rvecs.begin()+1,rvecs.end());
            //tvecs.erase(tvecs.begin()+1,tvecs.end());
            temp=
                    calibrateCamera(objectPoints, // the 3D points
                                    imagePoints,  // the image points
                                    Size(width,height),    // image size
                                    cam, // output camera matrix
                                    dist,   // output distortion matrix
                                    rvecs, tvecs, // Rs, Ts
                                    CV_CALIB_USE_INTRINSIC_GUESS+CV_CALIB_FIX_INTRINSIC);
            qDebug()<<temp;
            Rodrigues(rvecs[0],rotationMatrix);
            translationVector=tvecs[0];

        }
    }else{
        string file=folder+"/frame"+boost::lexical_cast<string>(0)+".bmp";
        img=imread(file.c_str());
        Mat imgboard=img;
        Size boardSize(5,4);
        addChessboardPoints(img, boardSize,imgboard) ;
        if(objectPoints.size()>0 &&imagePoints.size()>0)
        {
            double temp=
                    calibrateCamera(objectPoints, // the 3D points
                                    imagePoints,  // the image points
                                    Size(640,480),    // image size
                                    cameraMatrix, // output camera matrix
                                    distCoeffs,   // output distortion matrix
                                    rvecs, tvecs, // Rs, Ts
                                    CV_CALIB_USE_INTRINSIC_GUESS+CV_CALIB_FIX_INTRINSIC);
        }
        Rodrigues(rvecs[0],rotationMatrix);
        translationVector=tvecs[0];
        computeWorld();
        line( imgboard, world[device][0], world[device][1], Scalar(0, 255, 0), 2);
        line( imgboard, world[device][0], world[device][2], Scalar(255, 0, 0), 2 );
        line( imgboard, world[device][0], world[device][3], Scalar(0, 0, 255), 2 );
        cvtColor(imgboard, imgboard, CV_BGR2RGB);
        QImage qimg= QImage((const unsigned char*)(imgboard.data),
                            imgboard.cols,imgboard.rows,imgboard.step,QImage::Format_RGB888);
        emit this->intrCalibrationSuccess(qimg,device);



    }

   // string file=folder+"/frame"+boost::lexical_cast<string>(0)+".bmp";
  //  img=imread(file.c_str());
  //  Mat undist=remap(img);
  //  file="frame_undist"+boost::lexical_cast<string>(device)+".bmp";
   // imwrite(file.c_str(),undist);
       displayMatrix(cameraMatrix);
       //displayMatrix(distCoeffs);
    displayMatrix(rotationMatrix);
    storeMatrices();
    stopped=true;
    emit intrCalibStop(this->device,true);



}
void IntrCalibThread::stop()
{
    cap.release();
    stopped=true;
}
bool IntrCalibThread::isStopped()
{
    return stopped;
}

// Add scene points and corresponding image points
void IntrCalibThread::addPoints(const vector<Point2f>& imageCorners,
                                const vector<Point3f>& objectCorners) {


    // 2D image points from one view
    imagePoints.push_back(imageCorners);
    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

// Open chessboard images and extract corner points
bool IntrCalibThread::addChessboardPoints(const Mat &img,Size & boardSize,Mat &imgCam1)
{
    // the points on the chessboard
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;
    Mat image;
    cvtColor(img, image, CV_BGR2GRAY);
    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {

            objectCorners.push_back(Point3f(i, j, 0.0f));
        }
    }
    // Get the chessboard corners
    bool found = findChessboardCorners(image, boardSize, imageCorners);
    drawChessboardCorners(imgCam1,boardSize, imageCorners,found);
    if(found){
        // Get subpixel accuracy on the corners
        cornerSubPix(image, imageCorners,
                     Size(10,10),
                     Size(-1,-1),
                     TermCriteria(cv::TermCriteria::MAX_ITER +
                                  cv::TermCriteria::EPS,
                                  30,		// max number of iterations
                                  0.01));     // min accuracy

        // If we have a good board, add it to our data
        if (int(imageCorners.size() )== int(boardSize.area())) {

            // Add image and scene points from one view
            addPoints(imageCorners, objectCorners);
        }
    }


    return found;
}
void  IntrCalibThread::displayMatrix(Mat& mat)
{
    qDebug()<<"display";
    qDebug() << mat.rows << "x" << mat.cols << endl;
    qDebug()<< "[";
    for (int i=0 ;i<mat.rows;i++)
    {
        for (int j=0 ;j<mat.cols;j++)
        {
            qDebug()<< mat.at<double>(i,j) << " ";
        }
        qDebug()<< endl;
    }
    qDebug()<< "]";
}
Mat IntrCalibThread::remap(const Mat &image) {

    Mat undistorted;

    if (mustInitUndistort) { // called once per calibration

        initUndistortRectifyMap(
                cameraMatrix,  // computed camera matrix
                distCoeffs,    // computed distortion matrix
                Mat(),     // optional rectification (none)
                Mat(),     // camera matrix to generate undistorted
                //            cv::Size(640,480),
                Size(image.size().width,image.size().height),  // size of undistorted
                CV_64FC1,      // type of output map
                map1, map2);   // the x and y mapping functions

        mustInitUndistort= false;
    }

    // Apply mapping functions
    cv::remap(image, undistorted, map1, map2,
              INTER_LINEAR); // interpolation type

    return undistorted;
}
bool IntrCalibThread::storeMatrices()
{
    if (!cameraMatrix.empty() &&!distCoeffs.empty() &&!rotationMatrix.empty() &&!translationVector.empty())
    {
        string filename=boost::lexical_cast<string>(this->device);
        QString s=QString::fromStdString("calib_data");
        if(!QDir("Folder").exists(s))
            QDir().mkdir(s);
        FileStorage fs("calib_data/cam"+filename+"Intrinsics.yml", FileStorage::WRITE);
        time_t rawtime; time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));
        fs<< "cameraMatrix" << cameraMatrix;
        fs<<"DistCoeffs"<< distCoeffs;
        fs.release();

        fs.open("calib_data/cam"+filename+"Extrinsic.yml", FileStorage::WRITE);
        fs << "calibrationDate" << asctime(localtime(&rawtime));
        fs<<"translationVector"<< translationVector;
        fs<<"rotationMatrix"<<rotationMatrix;
        fs.release();
        return true;
    }else return false;
}
void IntrCalibThread::readCamPar(){

    string path="/usr/not-backed-up/Dropbox/PhDmed/code/shape from shading/calib_data/";
    Mat mat1,mat2,mat3,mat4;


    string fileE= path+"cam"+boost::lexical_cast<std::string>(this->device)+"Extrinsic.yml";
    string fileI= path+"cam"+boost::lexical_cast<std::string>(this->device)+ "Intrinsics.yml";
    readMat(fileE.c_str(),
            mat1,mat2,"rotationMatrix","translationVector");
    readMat(fileI.c_str(),
            mat3,mat4,"cameraMatrix","DistCoeffs");
    this->rotationMatrix=mat1;
    this->translationVector=mat2;
    this->cameraMatrix=mat3;
    this->distCoeffs=mat4;
}
void IntrCalibThread::computeWorld()
{
    Mat aux=combineMat(this->rotationMatrix,this->translationVector,4,3);
    Mat Mwc = this->cameraMatrix*aux;
    double Owdata[] = {0.0, 0.0, 0.0, 1};
    Mat Ow = Mat(4, 1, 6, Owdata).clone();
    double NwxData[]={10, 0, 0, 1};
    Mat Nwx=Mat(4, 1, 6, NwxData).clone();
    double NwyData[]={0, 10, 0, 1};
    Mat Nwy=Mat(4, 1, 6, NwyData).clone();
    double NwzData[]={0, 0, 10, 1};
    Mat Nwz=Mat(4, 1, 6, NwzData).clone();
    Mat Oi=Mwc*Ow;
    Oi=Oi/Oi.at<double>(0,2);
    world[this->device][0]=Point2f(Oi.at<double>(0,0),Oi.at<double>(0,1));
    Mat Nix = Mwc*Nwx;
    Nix = Nix/Nix.at<double>(0,2);
    world[this->device][1]=Point2f(Nix.at<double>(0,0),Nix.at<double>(0,1));
    Mat Niy = Mwc*Nwy;
    Niy = Niy/Niy.at<double>(0,2);
    world[this->device][2]=Point2f(Niy.at<double>(0,0),Niy.at<double>(0,1));
    Mat Niz = Mwc*Nwz;
    Niz = Niz/Niz.at<double>(0,2);
    world[this->device][3]=Point2f(Niz.at<double>(0,0),Niz.at<double>(0,1));;


}
void IntrCalibThread::readMat(const string& filename, Mat & mat1,Mat & mat2,const string & str1,const string &str2)
{
    FileStorage fs(filename, FileStorage::READ);
    if (fs.isOpened()){
        fs[str1]>>mat1;
        fs[str2] >> mat2;
        qDebug()<< "matrix dimensions: " << mat1.rows<<"x"<<mat1.cols<< endl;
        qDebug()<< "matrix dimensions: " << mat2.rows<<"x"<<mat2.cols<< endl;
        // cout<<mat1<<endl;
        //cout<<mat2<<endl;
    }
    else{
        qDebug() <<"file :"<<filename.c_str()<<" could not be openned";
    }
    fs.release();

}
Mat IntrCalibThread::combineMat(const Mat & mat1,const Mat & mat2,int col,int row)
{
    Mat aux(row, col, DataType<double>::type);
    for(int i=0;i<mat1.rows;i++){
        for(int j=0; j<mat1.cols;j++){
            aux.at<double>(i,j)=mat1.at<double>(i,j);
        }
    }
    for(int i=0;i<mat2.rows;i++){
        for(int j=0; j<mat2.cols;j++){
            aux.at<double>(i,mat1.cols+j)=mat2.at<double>(i,j);
        }
    }

    return aux;
}
