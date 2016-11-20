#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv/cv.h>


/** @class Estimator
* Define several method to perform RANSAC\n
* This is a part of OpenCV source-code, rewrite for Version 2.1
*/
class Estimator
{
public :
    Estimator(int nbPoints);
    ~Estimator();

    int modelPoints; /**< number of point required to compute the model*/
    CvRNG rng; /**< OpenCV random integer generator*/
    bool checkPartialSubsets; /**< check that each point does not belong to a line connecting some previously selected points*/

    void setPointNumber(int nbPoints);

    bool runRANSAC( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, double reprojThreshold, double confidence, int maxIters = 2000);
    bool runRANSAC( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, cv::Mat &err, double reprojThreshold, double confidence, int maxIters = 2000);
    bool runLMeDS( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, double confidence, int maxIters = 2000 );
    int runKernel( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M );
    bool getSubset( cv::Mat &m1, cv::Mat &m2, cv::Mat &ms1, cv::Mat &ms2, int nb = 300);
    int findInliers( cv::Mat &m1, cv::Mat &m2, cv::Mat &M, cv::Mat &err, double reprojThreshold);
    int updateNumItersRANSAC( double p, double ep, int model_points, int max_iters );
    void computeReprojError( cv::Mat &_m1, cv::Mat &_m2, cv::Mat &model, cv::Mat &_err );

    int run7Point( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M );
    int run8Point( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M );
    int run11Point( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M );
    bool runNPoint( cv::Mat &m1, cv::Mat &m2, cv::Mat &M , int nbPoints = -1);
    bool runNPoint( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M , int nbPoints = -1);
    int runNPoint2( cv::Mat &m1, cv::Mat &m2, std::vector<cv::Mat> &M , int nbPoints = -1);
    int runNPoint2( cv::Mat &m1, cv::Mat &m2, cv::Mat &M , int nbPoints = -1);

    bool checkSubset( cv::Mat &m, int count );
    bool contains(std::vector<int> &vec, int &val);
/*
    bool runRANSACOriginal( const CvMat* m1, const CvMat* m2, CvMat* model, CvMat* mask0, double reprojThreshold, double confidence, int maxIters );
    int runKernelOriginal( const CvMat* m1, const CvMat* m2, CvMat* model );
    bool getSubsetOriginal( const CvMat* m1, const CvMat* m2, CvMat* ms1, CvMat* ms2, int maxAttempts );
    int findInliersOriginal( const CvMat* m1, const CvMat* m2, const CvMat* model, CvMat* _err, CvMat* _mask, double threshold );
    void computeReprojErrorOriginal( const CvMat* _m1, const CvMat* _m2, const CvMat* model, CvMat* _err );
    int run7PointOriginal( const CvMat* _m1, const CvMat* _m2, CvMat* _fmatrix );
    int run8PointOriginal( const CvMat* _m1, const CvMat* _m2, CvMat* _fmatrix );
    bool checkSubsetOriginal( const CvMat* m, int count );
*/

    //Display functions
    void displayMat(cv::Mat M, std::string name = "");
    void displayMat(CvMat *M, std::string name = "");
    void displayMatSize(cv::Mat M, std::string name = "");
    void displayMatSize(CvMat *M, std::string name = "");
    void display(double n, std::string name = "");
    void display(double n1,double n2, std::string name = "");
    void display(double n1,double n2,double n3, std::string name = "");

    //Math functions
    cv::Mat mean2(cv::Mat &M);
    cv::Mat mean3(cv::Mat &M);
    double normalize2(cv::Mat &M, cv::Mat m);
    double normalize3(cv::Mat &M, cv::Mat m);
    void kron(cv::Mat M1, cv::Mat M2, cv::Mat &M);
    cv::Mat kron(cv::Mat M1, cv::Mat M2);
    void cloneRow(cv::Mat &M1, int a, cv::Mat &M2, int b);
};

#endif // ESTIMATOR_H
