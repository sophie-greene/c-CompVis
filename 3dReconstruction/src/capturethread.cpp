#include "include/capturethread.h"

CaptureThread::CaptureThread(int d,int w,int h) :
        QThread()
{
    this->device=d;
    this->width=w;
    this->height=h;
    this->stopped=false;
    this->cap.open(this->device);
    if(cap.isOpened() )  // check if we succeeded
    {
        //change resolution and compression of v4l2
        cap.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
       // std::cout<<cap.get(CV_CAP_PROP_FRAME_WIDTH);
      // std::cout<<cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('Y', 'U', 'Y', 'V') );
        //        cap.set(CV_CAP_PROP_SATURATION,0.5);
        //        cap.set(CV_CAP_PROP_GAIN,0.5);
        //        cap.set(CV_CAP_PROP_CONTRAST,.5);
        //       cap.set(CV_CAP_PROP_BRIGHTNESS,.64);
        //       cap.set(CV_CAP_PROP_SHARPNESS,0.5);
        //cap.set(CV_CAP_PROP_WHITE_BALANCE,3724);
        this->threadCnt=0;
        this->deviceOpened=true;
    }
    else
    {
        this->deviceOpened=false;
        this->stopped=true;
    }
}

void CaptureThread::run()
{

    Mat imgCam,imgGray;
    int x;
    int y;
    int w=  cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int h=  cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    qDebug()<<w<<"x"<<h;
    while(!this->stopped){
        threadCnt=threadCnt+1;
        if(this->stopped)break;//thread ending while loop is running
        if(this->cap.isOpened()){
            this->cap.read(imgCam);
//            im=imgCam;
//            split()
           cvtColor(imgCam, imgGray, CV_RGB2GRAY);
           detectPoint(imgGray,x,y);

            circle(imgCam,cvPoint(x,y),2,cvScalar(255,0,0),4);
            cvtColor(imgCam, imgCam, CV_BGR2RGB);
            QImage qimg= QImage((const unsigned char*)(imgCam.data),
                                imgCam.cols,imgCam.rows,imgCam.step,QImage::Format_RGB888);

           // qDebug()<<"index befor emit"<<x<<" ,"<<y;
            emit this->laserAquired(this->device,this->threadCnt,x,y);
            emit this->imageAquired(qimg,this->device,x,y);

           // cvWaitKey();
           // msleep(200);
        }
        else
        {
            qDebug()<<"cam not open, video"<<this->device;
        }

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
void CaptureThread::detectPoint(const Mat & img,int & x,int & y)
{
    Point pt;
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
void  CaptureThread::displayMatrix(const Mat& mat)
{

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




void CaptureThread::readCamPar(){

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
void CaptureThread::computeWorld()
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
void CaptureThread::readMat(const string& filename, Mat & mat1,Mat & mat2,const string & str1,const string &str2)
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
Mat CaptureThread::combineMat(const Mat & mat1,const Mat & mat2,int col,int row)
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
