#ifndef MUTUALINFORMATION_H
#define MUTUALINFORMATION_H

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

/** @class MI
* Define several method to get correspondences between two images using Mutual Information
*/
class MI : public cv::Feature2D
{

};

typedef MI MIFeatureDetector;
typedef MI MIDescriptorExtractor;
    
class MutualInformation
{
public:
  MutualInformation(int nbBins = 128);
  MutualInformation(cv::Mat image1, cv::Mat image2, int nbBins = 128);

  double compute(cv::Mat image1, cv::Mat image2, int nbBins = -1);
  double probability(cv::Mat &I, int t);
  double joinProbability(cv::Mat &I1, cv::Mat &I2, int i, int j);
  double probability(std::vector<int> &I, int t,int nb);
  double joinProbability(std::vector<int> &I12, int i, int j);
  double probability(int I[], int t,int nb);
  double joinProbability(int I12[], int i, int j, int nb);

  double entropy(cv::Mat I);
  double joinEntropy(cv::Mat &I1, cv::Mat &I2, int i, int j);

  void scaleBins(cv::Mat &I, int nbBins, int oldNbBins = 256);
  double probability(cv::Mat I, int t, cv::Mat p);
  double d(int p);
  cv::Mat w(cv::Mat I, cv::Mat p);

  cv::Mat G(cv::Mat I1, cv::Mat I2);
  cv::Mat H(cv::Mat I1, cv::Mat I2);
  cv::Mat computePose(cv::Mat I1, cv::Mat I2);

  cv::Mat dpii() {return cv::Mat();}
  cv::Mat d2pii() {return cv::Mat();}

  double testSSD(cv::Mat image1, cv::Mat image2);
private:
  int nbBins_;
};


#endif
