#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

#include "MCharImage.h"
#include "MDetecteur.h"
#include "MListePI.h"
#include "MListeCouples.h"
#include "MUtils.h"

/*struct PointData{
  cv::Point2d pt;
  CvSeq *descriptorSeq;
  cv::Mat descriptorMat;
};*/

/** @class Detector
* Define several method to get correspondences between two images\n
* This is a part of OpenCV samples
*/
class Detector
{
public:
    inline bool toBool(double val){ if(val==0) return false; else return true;} /**< Convert double to bool*/
    enum FeatureType {FAST,STAR,SIFT,SURF,OLDSURF,ORB,MSER,GFTT,HARRIS,DENSE,BLOB,NO_FEATURE_TYPE}; /**< Feature types from OpenCV and Eric Royer (HARRIS)*/
    enum DescriptorType {SIFT_EXTRACTOR,SURF_EXTRACTOR,ORB_EXTRACTOR,BRIEF_EXTRACTOR,NO_DESCRIPTOR_TYPE}; /**< Descriptor types from OpenCV*/
    enum MatcherType {L1,L2,HAMMING1,HAMMING2,FLANN,NO_MATCHER_TYPE}; /**< MatcherTyper from OpenCV*/

    Detector();
    Detector(FeatureType detectorType, DescriptorType descriptorType, MatcherType matcherType);
    Detector(std::string file1, std::string file2, int flip = 0);
    Detector(cv::Mat im1, cv::Mat im2, int flip = 0);
    ~Detector();
    void init();

    static FeatureType getFeatureType(std::string type);
    static DescriptorType getDescriptorType(std::string type);
    static MatcherType getMatcherType(std::string type);
    static void displayParametersHelp();

    void setType(FeatureType featureType = NO_FEATURE_TYPE, DescriptorType descriptorType = NO_DESCRIPTOR_TYPE, MatcherType matcherType = NO_MATCHER_TYPE);

    std::vector<cv::Point2d> getPoints(int index);
    std::vector<cv::Point2d> getAllPoints(int index);
    std::vector< std::pair<int,int> > getCorrespondences();
    local_vis_eric::MListeCouples getCouples();

    int getNbPairs();
    cv::Mat getResult(int dimMax = 1280, bool displayAll = true);
    cv::Mat getResult(std::vector<bool> good_matching, int dimMax = 1280, bool displayAll = true);
    cv::Mat getImage(int index);

    void saveResult(std::string filename, int dimMax = 1280);
    void saveResult(std::string filename, std::vector<bool> good_matching, int dimMax = 1280);
    void displayResult(int dimMax = 1280, bool wait = true);
    void displayResult(std::vector<bool> good_matching, int dimMax = 1280, bool wait = true);

    void setImages(cv::Mat &im1, cv::Mat &im2);
    void setImages(std::string file1, std::string file2);

    void setPoints(std::vector<cv::Point2d> pts, int index);

    void setDefaultParameters(FeatureType type);
    void setParam(int nbParam, double p1=0.0, double p2=0.0, double p3=0.0, double p4=0.0, double p5=0.0, double p6=0.0, double p7=0.0, double p8=0.0, double p9=0.0, double p10=0.0);
    void setParam(std::vector<double> params);

    void flipImages(int mode);

    void compute(FeatureType detectorType, DescriptorType descriptorType, MatcherType matcherType);
    void compute();
    void computeHarris();
    void computeSURF();

    std::vector<cv::Point2d> getPoints(cv::Mat I);
    std::vector<cv::Point2d> getPointsHarris(cv::Mat I);
    std::vector<cv::Point2d> getPointsSURF(cv::Mat I);

protected:
    std::vector<cv::Point2d> pts1; /**< pts1 is points vector storage for the first image*/
    std::vector<cv::Point2d> pts2; /**< pts2 is points vector storage for the second image*/
    std::vector<cv::Point2d> pts1_all; /**< pts1_all is points vector storage for the first image*/
    std::vector<cv::Point2d> pts2_all; /**< pts2_all is points vector storage for the second image*/
    cv::Mat descriptors_img1; /**< descriptors_img1 is points descriptor storage for the first image*/
    cv::Mat descriptors_img2; /**< descriptors_img1 is points descriptor storage for the second image*/
    CvSeq *keypoints_img1_old; /**< keypoint_img1_old is keypoints storage for the first image*/
    CvSeq *keypoints_img2_old; /**< keypoint_img2_old is keypoints storage for the second image*/
    CvSeq *descriptors_img1_old; /**< descriptors_img1_old is descriptors storage for the first image*/
    CvSeq *descriptors_img2_old; /**< descriptors_img2_old is descriptors storage for the second image*/
    CvMemStorage *storage1; /**< storage1 is memory storage for the first image*/
    CvMemStorage *storage2; /**< storage2 is memory storage for the second image*/
    std::vector<int> ptpairs1; /**< ptpairs1 is pair storage for the first and second images*/
    cv::Mat img1; /**< img1 is the first image*/
    cv::Mat img2; /**< img2 is the second image*/
    std::vector< std::pair<int,int> > correspondences; /**< correspondences1 between image1 and image2*/

    void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors, const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, std::vector<int>& ptpairs );
    int naiveNearestNeighbor( const float* vec, int laplacian, const CvSeq* model_keypoints, const CvSeq* model_descriptors );
    double compareSURFDescriptors( const float* d1, const float* d2, double best, int length );

    std::vector<cv::Point2d> extractPoints(CvSeq* seq);

private:
    FeatureType currentFeatureType;
    DescriptorType currentDescriptorType;
    MatcherType currentMatcherType;

    double parameters[10];
};

#endif // DETECTOR_H
