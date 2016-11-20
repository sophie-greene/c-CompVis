#include "cameracalibrator.h"

// Open chessboard images and extract corner points
int CameraCalibrator::addChessboardPoints(
        const vector<string>& filelist,
        Size & boardSize) {

    // the points on the chessboard
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;

    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {

            objectCorners.push_back(Point3f(i, j, 0.0f));
        }
    }

    // 2D Image points:
    Mat image; // to contain chessboard image
    int successes = 0;
    // for all viewpoints
    for (int i=0; i<filelist.size(); i++) {
        qDebug()<<i;
        qDebug()<<filelist[i].c_str();
        // Open the image
        image = imread(filelist[i],0);

        // Get the chessboard corners
        bool found = findChessboardCorners(
                image, boardSize, imageCorners);
        if(found){
        // Get subpixel accuracy on the corners
        cornerSubPix(image, imageCorners,
                     Size(5,5),
                     Size(-1,-1),
                     TermCriteria(TermCriteria::MAX_ITER +
                                  TermCriteria::EPS,
                                  30,		// max number of iterations
                                  0.1));     // min accuracy

        // If we have a good board, add it to our data
        if (imageCorners.size() == boardSize.area()) {

            // Add image and scene points from one view
            addPoints(imageCorners, objectCorners);
            successes++;
        }}

    }

    return successes;
}

// Add scene points and corresponding image points
void CameraCalibrator::addPoints(const vector<Point2f>& imageCorners,
                                 const vector<Point3f>& objectCorners) {

    // 2D image points from one view
    imagePoints.push_back(imageCorners);
    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

// Calibrate the camera
// returns the re-projection error
double CameraCalibrator::calibrate(Size &imageSize)
{
    // undistorter must be reinitialized
    mustInitUndistort= true;

    //Output rotations and translations
    vector<Mat> rvecs, tvecs;

    // start calibration
    return
            calibrateCamera(objectPoints, // the 3D points
                            imagePoints,  // the image points
                            imageSize,    // image size
                            cameraMatrix, // output camera matrix
                            distCoeffs,   // output distortion matrix
                            rvecs, tvecs, // Rs, Ts
                            flag);        // set options
    //					,CV_CALIB_USE_INTRINSIC_GUESS);

}

// remove distortion in an image (after calibration)
Mat CameraCalibrator::remap(const Mat &image) {

    Mat undistorted;

    if (mustInitUndistort) { // called once per calibration

        initUndistortRectifyMap(
                cameraMatrix,  // computed camera matrix
                distCoeffs,    // computed distortion matrix
                Mat(),     // optional rectification (none)
                Mat(),     // camera matrix to generate undistorted
                Size(image.size().width,image.size().height),
                //            image.size(),  // size of undistorted
                CV_32FC1,      // type of output map
                map1, map2);   // the x and y mapping functions

        mustInitUndistort= false;
    }

    // Apply mapping functions
    cv::remap(image, undistorted, map1, map2,
              cv::INTER_LINEAR); // interpolation type
imwrite("s.jpg",undistorted);
    return undistorted;
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
    if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}

