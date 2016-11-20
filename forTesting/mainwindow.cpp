#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_pushButton_clicked()
{
    string path="/export/mailgrp2_d/scssaq/captureVideo";
//    vector<string> filelst[8];
//    for (int cam=0;cam<8;cam++)
//    {
//        for (int light=0;light<8;light++)
//            filelst[cam].push_back(path+"cam"
//                              +boost::lexical_cast<std::string>(cam)+
//                              "light"+boost::lexical_cast<std::string>(light)+
//                              "pos0.bmp");
        Mat img,rotMat,transVec,camMat,distCoeffs;
        img=imread(path+"/cam5.bmp");
        qDebug()<<img.cols;
        readMat("/export/mailgrp2_d/scssaq/captureVideo/calib_data/cam5Extrinsic.yml",
                rotMat,transVec,"rotationMatrix","translationVector");
        readMat("/export/mailgrp2_d/scssaq/captureVideo/calib_data/cam5Intrinsics.yml",
                camMat,distCoeffs,"cameraMatrix","DistCoeffs");
      //  Undistort(filelst[cam],camMat,distCoeffs);
            Mat imUndist;
            undistort(img,imUndist,camMat,distCoeffs, Mat());
            Mat rect;
            warpPerspective(imUndist, rect, rotMat, Size(640,480),INTER_LINEAR, BORDER_CONSTANT, 0);
            namedWindow("undistorted",2);
            //    namedWindow("origional",1);
            //     imshow( "origional", img );
            imshow( "undistorted", imUndist );
//    }



    //    Mat imUndist=remap(img,camMat,distCoeffs );
    //    namedWindow("undistorted",2);
    //    //    namedWindow("origional",1);
    //    //     imshow( "origional", img );
    //    imshow( "undistorted", imUndist );
    //
    //    perspectiveTransform(img, dst, rotMat);
    //    warpPerspective(img, dst,M, dsize, INTER_LINEAR,BORDER_CONSTANT, Scalar());
    //    detect("/export/mailgrp2_d/scssaq/lightDirectionSnaps/video7.jpg",
    //    "/export/mailgrp2_d/scssaq/lightDirectionSnaps/video7flat.jpg");

}
void MainWindow::readMat(const string& filename, Mat & mat1,Mat & mat2,const string & str1,const string &str2)
{
    FileStorage fs(filename, FileStorage::READ);
    if (fs.isOpened()){
        fs[str1]>>mat1;
        fs[str2] >> mat2;
        qDebug()<< "matrix dimensions: " << mat1.rows<<"x"<<mat1.cols<< endl;
    }
    else{
        qDebug() <<"file :"<<filename.c_str()<<" could not be openned";
    }
    fs.release();

}
void MainWindow::Undistort(vector <string>&filelist,const Mat & camMat,const Mat & distCoeffs)
{
     string path="/export/mailgrp2_d/scssaq/video/simpleimagecopier/pmcalib/";
    QString fold=QString::fromStdString(path+"undistorted");
    if(!QDir("Folder").exists(fold))
        QDir().mkdir(fold);
    for (int i=0;i<filelist.size();i++)
    {
        Mat image= imread(filelist[i].c_str());
        Mat undist=remap(image,camMat,distCoeffs );
        string s=filelist[i];
        qDebug()<<s.c_str();
        imwrite(s.c_str(),undist);
    }
}

Mat MainWindow::remap(const Mat & image, const Mat & cameraMatrix,const Mat & distCoeffs ) {
    cout <<cameraMatrix<<endl;
    cout<<distCoeffs<<endl;
    Mat undistorted;
    Mat map1,map2;
    //if (mustInitUndistort) { // called once per calibration

    initUndistortRectifyMap(
            cameraMatrix,  // computed camera matrix
            distCoeffs,    // computed distortion matrix
            Mat(),     // optional rectification (none)
            Mat(),     // camera matrix to generate undistorted
            //            cv::Size(640,480),
            Size(image.size().width,image.size().height),  // size of undistorted
            CV_32FC1,      // type of output map
            map1, map2);   // the x and y mapping functions
    cout <<map1.cols<<endl;
    cout <<map2.rows<<endl;
    //mustInitUndistort= false;
    //}

    // Apply mapping functions
    cv::remap(image, undistorted, map1, map2,
              cv::INTER_CUBIC); // interpolation type

    return undistorted;
}
void MainWindow::detect(const string & im1,const string & im2)
{


    Mat img_object = imread( im1, CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_scene = imread( im2, CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_object.data || !img_scene.data )
    { std::cout<< " --(!) Error reading images " << std::endl;}

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector( minHessian );

    std::vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( img_object, keypoints_object );
    detector.detect( img_scene, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
        { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);
    //namedWindow("hello",4);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );

    waitKey(0);

}

/** @function readme */
void readme()
{ std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
