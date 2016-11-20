#ifndef DETECTOR3_H
#define DETECTOR3_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>


/** @class Detector
* Define several method to get correspondences between two images\n
* This is a part of OpenCV samples
*/
class Detector3
{
public:
    Detector3();
    Detector3(std::string file1, std::string file2, std::string file3, int flip = 0);
    Detector3(cv::Mat im1, cv::Mat im2, cv::Mat im3, int flip = 0);
    ~Detector3();
    void init();

    std::vector<cv::Point2d> getPoints(int index);
    std::vector<cv::Point2d> getAllPoints(int index);
    std::vector< std::pair<int,int> > getCorrespondences(int index = 1);

    int getNbPairs();
    cv::Mat getResult(int dimMax = 1280);
    cv::Mat getResult(std::vector<bool> good_matching, int dimMax = 1280);
    cv::Mat getImage(int index);

    void saveResult(std::string filename, int dimMax = 1280);
    void saveResult(std::string filename, std::vector<bool> good_matching, int dimMax = 1280);
    void displayResult(int dimMax = 1280, bool wait = true);
    void displayResult(std::vector<bool> good_matching, int dimMax = 1280, bool wait = true);

    void setImages(cv::Mat &im1, cv::Mat &im2);
    void setImages(cv::Mat &im1, cv::Mat &im2, cv::Mat &im3);

    void setPoints(std::vector<cv::Point2d> pts, int index);

    void setParamsSIFT(int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6);
    void setParamsSURF(double hessianThreshold = 5000.0, int nOctaves = 4, int nOctaveLayers = 2, bool extended = false);
    void setDefaultParameters();

    void computeSIFT(bool useBruteForce = true);
    void computeSURF(bool useBruteForce = true);
    void computeMI(bool useBruteForce);
    void computeSURF_old();

    std::vector<cv::Point2d> getSurfOld(cv::Mat I);
    std::vector<cv::Point2d> getSurf(cv::Mat I);
    std::vector<cv::Point2d> getSift(cv::Mat I);

    void permute();
    void permute_old();

    bool getCorrespondingPoint(std::vector<cv::Point2d> &l1, std::vector<cv::Point2d> &l2, cv::Point2d &p1, cv::Point2d &p2);

protected:
    std::vector<cv::Point2d> pts1; /**< pts1 is points vector storage for the first image*/
    std::vector<cv::Point2d> pts2; /**< pts2 is points vector storage for the second image*/
    std::vector<cv::Point2d> pts3; /**< pts2 is points vector storage for the third image*/
    std::vector<cv::Point2d> pts1_all; /**< pts1_all is points vector storage for the first image*/
    std::vector<cv::Point2d> pts2_all; /**< pts2_all is points vector storage for the second image*/
    std::vector<cv::Point2d> pts3_all; /**< pts2_all is points vector storage for the third image*/
    cv::Mat descriptors_img1; /**< descriptors_img1 is points descriptor storage for the first image*/
    cv::Mat descriptors_img2; /**< descriptors_img1 is points descriptor storage for the second image*/
    cv::Mat descriptors_img3; /**< descriptors_img1 is points descriptor storage for the third image*/
    CvSeq *keypoints_img1_old; /**< keypoint_img1_old is keypoints storage for the first image*/
    CvSeq *keypoints_img2_old; /**< keypoint_img2_old is keypoints storage for the second image*/
    CvSeq *keypoints_img3_old; /**< keypoint_img3_old is keypoints storage for the third image*/
    CvSeq *descriptors_img1_old; /**< descriptors_img1_old is descriptors storage for the first image*/
    CvSeq *descriptors_img2_old; /**< descriptors_img2_old is descriptors storage for the second image*/
    CvSeq *descriptors_img3_old; /**< descriptors_img3_old is descriptors storage for the third image*/
    CvMemStorage *storage1; /**< storage1 is memory storage for the first image*/
    CvMemStorage *storage2; /**< storage2 is memory storage for the second image*/
    CvMemStorage *storage3; /**< storage3 is memory storage for the third image*/
    std::vector<int> ptpairs1; /**< ptpairs1 is pair storage for the first and second images*/
    std::vector<int> ptpairs2; /**< ptpairs2 is pair storage for the second and third images*/
    cv::Mat img1; /**< img1 is the first image*/
    cv::Mat img2; /**< img2 is the second image*/
    cv::Mat img3; /**< img2 is the third image*/
    std::vector< std::pair<int,int> > correspondences1; /**< correspondences1 between image1 and image2*/
    std::vector< std::pair<int,int> > correspondences2; /**< correspondences2 between image2 and image3*/

    void flipImages(int mode);

    void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors, const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, std::vector<int>& ptpairs );
    int naiveNearestNeighbor( const float* vec, int laplacian, const CvSeq* model_keypoints, const CvSeq* model_descriptors );
    double compareSURFDescriptors( const float* d1, const float* d2, double best, int length );

    std::vector<cv::Point2d> extractPoints(CvSeq* seq);

private:
    int _nfeatures;
    int _nOctaveLayersSIFT;
    double _contrastThreshold;
    double _edgeThreshold;
    double _sigma;
    double _hessianThreshold;
    int _nOctaves;
    int _nOctaveLayersSURF;
    bool _extended;
};

#endif // DETECTOR3_H
