#include "../include/detector3.h"

/** Load two images from files
 * @param file1,file2,file3 are input filenames
 * @param flip flip input images (in binary, to flip 1&2 use b011=3)
 */
Detector3::Detector3(std::string file1, std::string file2, std::string file3, int flip)
{
  img1 = cv::imread(file1.c_str());
  img2 = cv::imread(file2.c_str());
  img3 = cv::imread(file3.c_str());
  flipImages(flip);
  init();
}

/** Load two images from arguments
 * @param im1,im2,im3 are input images
 * @param flip flip input images (in binary, to flip 1&2 use b011=3)
 */
Detector3::Detector3(cv::Mat im1, cv::Mat im2, cv::Mat im3, int flip)
{
  img1 = im1;
  img2 = im2;
  img3 = im3;
  flipImages(flip);
  init();
}

Detector3::Detector3()
{
  std::cout << "Detector with 3 images" << std::endl;
  init();
}

Detector3::~Detector3()
{
  cvReleaseMemStorage(&storage1);
  cvReleaseMemStorage(&storage2);
  cvReleaseMemStorage(&storage3);
}

/** Initialize variables
 */
void Detector3::init()
{
  setDefaultParameters();
  keypoints_img1_old = 0;
  keypoints_img2_old = 0;
  keypoints_img3_old = 0;
  descriptors_img1_old = 0;
  descriptors_img2_old = 0;
  descriptors_img3_old = 0;
  storage1 = cvCreateMemStorage();
  storage2 = cvCreateMemStorage();
  storage3 = cvCreateMemStorage();
}

/** Flip images
 * @param mode flip images (in binary, ex: to flip 1&2 use 011 -> 3, use 7 for all)
 */
void Detector3::flipImages(int mode)
{
  if(mode==1 || mode==3 || mode==5 || mode==7)
    cv::flip(img1,img1,1);
  if(mode==2 || mode==3 || mode==6 || mode==7)
    cv::flip(img2,img2,1);
  if(mode==4 || mode==5 || mode==6 || mode==7)
    cv::flip(img3,img3,1);
}

/** Clone two images to use in feature detector
 * @param im1,im2,im3 are input images
 */
void Detector3::setImages(cv::Mat &im1, cv::Mat &im2, cv::Mat &im3)
{
  img1 = im1.clone();
  img2 = im2.clone();
  img3 = im3.clone();
}

/** Set SURF parameters
 * @param hessianThreshold, nOctaves, nOctaveLayers, extended are input parameters
 */
void Detector3::setParamsSURF(double hessianThreshold, int nOctaves, int nOctaveLayers, bool extended)
{
  _hessianThreshold = hessianThreshold;
  _nOctaves = nOctaves;
  _nOctaveLayersSURF = nOctaveLayers;
  _extended = extended;
}

/** Set SIFT parameters
 * @param nfeatures,nOctaveLayers,contrastThreshold,edgeThreshold,sigma are input detector parameters
 */
void Detector3::setParamsSIFT(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma)
{
  _nfeatures=nfeatures; _nOctaveLayersSIFT=nOctaveLayers; _contrastThreshold=contrastThreshold; _edgeThreshold=edgeThreshold; _sigma=sigma;
}

/** Reset default parameters\n
 *    //SIFT parameters\n
 *     _nfeatures=0; _nOctaveLayersSIFT=3; _contrastThreshold=0.04; _edgeThreshold=10; _sigma=1.6;\n
 *    //SURF parameters\n
 *    _hessianThreshold = 5000.0; _nOctaves = 4; _nOctaveLayersSURF = 2; _extended = false;
 */
void Detector3::setDefaultParameters()
{
  //SIFT
  _nfeatures=0; _nOctaveLayersSIFT=3; _contrastThreshold=0.04; _edgeThreshold=10; _sigma=1.6;
  //SURF
  _hessianThreshold = 5000.0; _nOctaves = 4; _nOctaveLayersSURF = 2; _extended = false;
}

/** Extract points using SIFT from two images and store them in two vectors
 * @param useBruteForce true if use 'cv::BruteForceMatcher', false for 'cv::FlannBasedMatcher'
 */
void Detector3::computeSIFT(bool useBruteForce)
{
  cv::SIFT detector(_nfeatures, _nOctaveLayersSIFT, _contrastThreshold, _edgeThreshold, _sigma);
  std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;

  detector.detect(img1, keypoints1);
  detector.detect(img2, keypoints2);
  detector.detect(img3, keypoints3);

  // computing descriptors
  cv::Mat descriptors1, descriptors2, descriptors3;
  detector.compute(img1, keypoints1, descriptors1);
  detector.compute(img2, keypoints2, descriptors2);
  detector.compute(img3, keypoints3, descriptors3);

  // matching descriptors
  std::vector< cv::DMatch > matches1, matches2, matches3;
  if(useBruteForce){
    cv::BFMatcher matcher(cv::NORM_L2);
    matcher.create("BruteForce");
    matcher.match(descriptors1, descriptors2, matches1);
    matcher.match(descriptors2, descriptors3, matches2);
    matcher.match(descriptors3, descriptors1, matches3);
  }else{
    cv::FlannBasedMatcher matcher;
    matcher.create("FlannBased");
    matcher.match(descriptors1, descriptors2, matches1);
    matcher.match(descriptors2, descriptors3, matches2);
    matcher.match(descriptors3, descriptors1, matches3);
  }

  // save matches
  pts1.clear();
  pts2.clear();
  pts3.clear();
  correspondences1.clear();
  correspondences2.clear();

  for(int i=0;i<matches1.size();i++){
    int index1 = matches1.at(i).queryIdx;
    int index2 = matches1.at(i).trainIdx;
    for(int j=0;j<matches2.size();j++){
      if(matches2.at(j).queryIdx == index2){
        int index3 = matches2.at(j).trainIdx;
        for(int k=0;k<matches3.size();k++){
          if(matches3.at(k).queryIdx == index3){
            if(matches3.at(k).trainIdx==index1){
                cv::KeyPoint kp1 = keypoints1[index1];
                cv::KeyPoint kp2 = keypoints2[index2];
                cv::KeyPoint kp3 = keypoints3[index3];
                cv::Point2d pt1 = cv::Point2f(kp1.pt);
                cv::Point2d pt2 = cv::Point2f(kp2.pt);
                cv::Point2d pt3 = cv::Point2f(kp3.pt);
                pts1.push_back(pt1);
                pts2.push_back(pt2);
                pts3.push_back(pt3);
                break;
            }
          }
        }
      }
    }
  }
}

/** Extract points using SURF from two images and store them in two vectors
 * @param useBruteForce true if use 'cv::BruteForceMatcher', false for 'cv::FlannBasedMatcher'
 */
void Detector3::computeSURF(bool useBruteForce)
{
  cv::SURF detector(_hessianThreshold,_nOctaves,_nOctaveLayersSURF,_extended);
  std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;

  detector.detect(img1, keypoints1);
  detector.detect(img2, keypoints2);
  detector.detect(img3, keypoints3);

  // computing descriptors
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors1, descriptors2, descriptors3;
  extractor.compute(img1, keypoints1, descriptors1);
  extractor.compute(img2, keypoints2, descriptors2);
  extractor.compute(img3, keypoints3, descriptors3);

  // matching descriptors
  std::vector< cv::DMatch > matches1, matches2, matches3;
  if(useBruteForce){
    cv::BFMatcher matcher(cv::NORM_L2);
    matcher.create("BruteForce");
    matcher.match(descriptors1, descriptors2, matches1);
    matcher.match(descriptors2, descriptors3, matches2);
    matcher.match(descriptors3, descriptors1, matches3);
  }else{
    cv::FlannBasedMatcher matcher;
    matcher.create("FlannBased");
    matcher.match(descriptors1, descriptors2, matches1);
    matcher.match(descriptors2, descriptors3, matches2);
    matcher.match(descriptors3, descriptors1, matches3);
  }

  // save matches
  pts1.clear();
  pts2.clear();
  pts3.clear();
  correspondences1.clear();
  correspondences2.clear();

  for(int i=0;i<matches1.size();i++){
    int index1 = matches1.at(i).queryIdx;
    int index2 = matches1.at(i).trainIdx;
    for(int j=0;j<matches2.size();j++){
      if(matches2.at(j).queryIdx == index2){
        int index3 = matches2.at(j).trainIdx;
        for(int k=0;k<matches3.size();k++){
          if(matches3.at(k).queryIdx == index3){
            if(matches3.at(k).trainIdx==index1){
                cv::KeyPoint kp1 = keypoints1[index1];
                cv::KeyPoint kp2 = keypoints2[index2];
                cv::KeyPoint kp3 = keypoints3[index3];
                cv::Point2d pt1 = cv::Point2f(kp1.pt);
                cv::Point2d pt2 = cv::Point2f(kp2.pt);
                cv::Point2d pt3 = cv::Point2f(kp3.pt);
                pts1.push_back(pt1);
                pts2.push_back(pt2);
                pts3.push_back(pt3);
                break;
            }
          }
        }
      }
    }
  }
}

/** Extract points using Mutual Information from two images and store them in two vectors
 * @warning TODO
 */
void Detector3::computeMI(bool useBruteForce)
{
  //cv::FeatureDetector detector();
  std::vector<cv::KeyPoint> keypoints1, keypoints2, keypoints3;

  //detector.detect(img1, keypoints1);
  //detector.detect(img2, keypoints2);
  //detector.detect(img3, keypoints3);

  // computing descriptors
  cv::Mat descriptors1, descriptors2, descriptors3;
  //detector.compute(img1, keypoints1, descriptors1);
  //detector.compute(img2, keypoints2, descriptors2);
  //detector.compute(img3, keypoints3, descriptors3);

  // matching descriptors
  std::vector< cv::DMatch > matches1, matches2, matches3;
  if(useBruteForce){
    cv::BFMatcher matcher(cv::NORM_L2);
    matcher.create("BruteForce");
    matcher.match(descriptors1, descriptors2, matches1);
    matcher.match(descriptors2, descriptors3, matches2);
    matcher.match(descriptors3, descriptors1, matches3);
  }else{
    cv::FlannBasedMatcher matcher;
    matcher.create("FlannBased");
    matcher.match(descriptors1, descriptors2, matches1);
    matcher.match(descriptors2, descriptors3, matches2);
    matcher.match(descriptors3, descriptors1, matches3);
  }

  // save matches
  pts1.clear();
  pts2.clear();
  pts3.clear();
  correspondences1.clear();
  correspondences2.clear();
  for(int i=0;i<matches1.size();i++){
    int index1 = matches1.at(i).queryIdx;
    int index2 = matches1.at(i).trainIdx;
    for(int j=0;j<matches2.size();j++){
      if(matches2.at(j).queryIdx == index2){
        int index3 = matches2.at(j).trainIdx;
        for(int k=0;k<matches3.size();k++){
          if(matches3.at(k).queryIdx == index3){
            if(matches3.at(k).trainIdx==index1){
                cv::KeyPoint kp1 = keypoints1[index1];
                cv::KeyPoint kp2 = keypoints2[index2];
                cv::KeyPoint kp3 = keypoints3[index3];
                cv::Point2d pt1 = cv::Point2f(kp1.pt);
                cv::Point2d pt2 = cv::Point2f(kp2.pt);
                cv::Point2d pt3 = cv::Point2f(kp3.pt);
                pts1.push_back(pt1);
                pts2.push_back(pt2);
                pts3.push_back(pt3);
                break;
            }
          }
        }
      }
    }
  }
}

/** Extract points using SURF from two images and store them in two vectors (old function)
 */
void Detector3::computeSURF_old()
{
  cv::Mat I1,I2,I3;
  if(img1.channels()==3) cv::cvtColor(img1,I1,CV_RGB2GRAY); else I1 = img1;
  if(img2.channels()==3) cv::cvtColor(img2,I2,CV_RGB2GRAY); else I2 = img2;
  if(img3.channels()==3) cv::cvtColor(img3,I3,CV_RGB2GRAY); else I3 = img3;

  IplImage gray1 = I1, gray2 = I2, gray3 = I3;

  CvSURFParams params = cvSURFParams(_hessianThreshold, 1);

  if(pts1_all.size()<10){
    cvExtractSURF( &gray1, 0, &keypoints_img1_old, &descriptors_img1_old, storage1, params );
    pts1_all = extractPoints(keypoints_img1_old);
  }
  if(pts2_all.size()<10){
    cvExtractSURF( &gray2, 0, &keypoints_img2_old, &descriptors_img2_old, storage2, params );
    pts2_all = extractPoints(keypoints_img2_old);
  }
  if(ptpairs1.size()<20){
    ptpairs1.clear();
    findPairs( keypoints_img1_old, descriptors_img1_old, keypoints_img2_old, descriptors_img2_old, ptpairs1 );
  }

  cvExtractSURF( &gray3, 0, &keypoints_img3_old, &descriptors_img3_old, storage3, params );
  pts3_all = extractPoints(keypoints_img3_old);

  ptpairs2.clear();
  findPairs( keypoints_img2_old, descriptors_img2_old, keypoints_img3_old, descriptors_img3_old, ptpairs2 );

  std::vector<int> ptpairs3;
  findPairs( keypoints_img1_old, descriptors_img1_old, keypoints_img3_old, descriptors_img3_old, ptpairs3 );

  // save matches
  pts1.clear();
  pts2.clear();
  pts3.clear();
  correspondences1.clear();
  correspondences2.clear();

  std::vector<cv::Point2d> pts1_temp, pts2_temp;
  std::vector<cv::Point2d> pts1_test, pts3_test;

  for(unsigned int n = 0; n < ptpairs3.size(); n += 2 ) {
    int index1 = ptpairs3[n];
    int index3 = ptpairs3[n+1];
    CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( keypoints_img1_old, index1 );
    CvSURFPoint* r3 = (CvSURFPoint*)cvGetSeqElem( keypoints_img3_old, index3 );
    cv::Point2d pt1(r1->pt);
    cv::Point2d pt3(r3->pt);
    pts1_test.push_back(pt1);
    pts3_test.push_back(pt3);
  }

  for(unsigned int n = 0; n < ptpairs1.size(); n += 2 ) {
    int index1 = ptpairs1[n];
    int index2 = ptpairs1[n+1];
    correspondences1.push_back( std::make_pair(index1,index2));
    CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( keypoints_img1_old, index1 );
    CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( keypoints_img2_old, index2 );
    cv::Point2d pt1(r1->pt);
    cv::Point2d pt2(r2->pt);
    pts1_temp.push_back(pt1);
    pts2_temp.push_back(pt2);
  }

  for(unsigned int n = 0; n < ptpairs2.size(); n += 2 ) {
    int index2 = ptpairs2[n];
    int index3 = ptpairs2[n+1];
    correspondences2.push_back( std::make_pair(index2,index3));
    CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( keypoints_img2_old, index2 );
    CvSURFPoint* r3 = (CvSURFPoint*)cvGetSeqElem( keypoints_img3_old, index3 );
    cv::Point2d pt2(r2->pt);
    cv::Point2d pt3(r3->pt);

    for( unsigned int i=0;i<pts1_temp.size();i++){
      if(pts2_temp.at(i)!=pt2)
        continue;
      cv::Point2d pt1 = pts1_temp.at(i);
      cv::Point2d p3temp;
      if(!getCorrespondingPoint(pts1_test,pts3_test,pt1,p3temp))
        continue;

      if(p3temp!=pt3){
        continue;
      }
      pts1.push_back(pt1);
      pts2.push_back(pt2);
      pts3.push_back(pt3);
    }
  }
}

/** Extract points from a sequence
 * @param seq is the input sequence
 * @retval vector of points
 */
std::vector<cv::Point2d> Detector3::extractPoints(CvSeq* seq)
{
  std::vector<cv::Point2d> pts;
  for(int i=0;i<seq->total;i++){
    CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( seq, i);
    cv::Point2d pt(r->pt);
    pts.push_back(pt);
  }
  return pts;
}

/** Return points in image 'index'
 * @param index image index [1:3]
 * @retval points vector in the image
 */
std::vector<cv::Point2d> Detector3::getPoints(int index)
{
  assert(index>=1 && index <=3);
  switch(index){
  case 1 : return pts1;
  case 2 : return pts2;
  case 3 : return pts3;
  default: return std::vector<cv::Point2d>();
  }
}

/** Return points in image 'index'
 * @param index image index [1:3]
 * @retval points vector in the image
 */
std::vector<cv::Point2d> Detector3::getAllPoints(int index)
{
  assert(index>=1 && index <=3);
  switch(index){
  case 1 : return pts1_all;
  case 2 : return pts2_all;
  case 3 : return pts3_all;
  default: return std::vector<cv::Point2d>();
  }
}

/** Return correspondences between image 'index' and inmage 'index+1'
 * @param index image index [1:2]
 * @retval correspondences vector
 */
std::vector< std::pair<int,int> > Detector3::getCorrespondences(int index)
{
  assert(index>=1 && index <=2);
  switch(index){
  case 1: return correspondences1;
  case 2: return correspondences2;
  default : return std::vector< std::pair<int,int> >();
  }
}

/** Set points in image 'index'
 * @param pts input list of 2D points
 * @param index image index [1:3]
 */
void Detector3::setPoints(std::vector<cv::Point2d> pts, int index)
{
  assert(index>=1 && index <=3);
  switch(index){
  case 1 : pts1 = pts; break;
  case 2 : pts2 = pts; break;
  case 3 : pts3 = pts; break;
  }
}

/** Return the number of correspondences
 * @retval min of pts1 and pts2 length
 */
int Detector3::getNbPairs()
{
  return pts1.size()<pts2.size()?pts1.size():pts2.size();
}

/** Get an image with all correspondences found
 * @param dimMax maximal width for the output image
 */
cv::Mat Detector3::getResult(int dimMax)
{
  std::vector<bool> good_matching;
  return getResult(good_matching,dimMax);
}

/** Get an image with all correspondences found
 * @param good_matching is input bool vector to specify if a pair of point is an inlier or outlier
 * @param dimMax maximal width for the output image
 */
cv::Mat Detector3::getResult(std::vector<bool> good_matching, int dimMax)
{
  cv::Mat color;
  cv::Size size( img1.cols + img2.cols + img3.cols, MAX(img1.rows, MAX(img2.rows, img3.rows)) );

  color.create( size, CV_MAKETYPE(img1.depth(), 3) );

  cv::Mat outImg1, outImg2, outImg3;
  outImg1 = color( cv::Rect(0, 0, img1.cols, img1.rows) );
  outImg2 = color( cv::Rect(img1.cols, 0, img2.cols, img2.rows) );
  outImg3 = color( cv::Rect(img1.cols+img2.cols, 0, img3.cols, img3.rows) );

  img1.copyTo(outImg1);
  img2.copyTo(outImg2);
  img3.copyTo(outImg3);

  //cv::cvtColor(gray,color,CV_GRAY2RGB);

  for(unsigned int i=0;i<pts1.size();i++){
    cv::Point2d pt1 = pts1.at(i);
    cv::Point2d pt2 = pts2.at(i) + cv::Point2d(img1.cols,0);
    cv::Point2d pt3 = pts3.at(i) + cv::Point2d(img1.cols+img2.cols,0);

    cv::Scalar linecolor = CV_RGB(255,255,255);
    bool drawline = true;
    if(i<good_matching.size()){
      if(good_matching.at(i)){
        linecolor = CV_RGB(0,255,0);
        drawline = false;
      }else{
        linecolor = CV_RGB(255,0,0);
      }
    }
    cv::circle( color, pt1, 3, CV_RGB(0,0,255), -1);
    cv::circle( color, pt2, 3, CV_RGB(0,0,255), -1);
    cv::circle( color, pt3, 3, CV_RGB(0,0,255), -1);

    if(drawline){
      cv::line( color, pt1, pt2, linecolor, 1);
      cv::line( color, pt2, pt3, linecolor, 1);
    }
  }

  if(size.width>dimMax){
    double scale = (double)color.cols/(double)dimMax;
    cv::resize(color,color,cv::Size(color.cols/scale,color.rows/scale));
  }
  return color;
}

/** Display an image with all correspondences found
 * @param dimMax maximal width for the output image
 * @param wait true if wait key
 */
void Detector3::displayResult(int dimMax, bool wait)
{
  cv::Mat color = getResult(dimMax);
  cv::imshow("Matching",color);
  if(wait) cv::waitKey(0);
  else cv::waitKey(10);
}

/** Display an image with all correspondences found
 * @param good_matching true for each good matching, else false
 * @param dimMax maximal width for the output image
 * @param wait true if wait keypress event
 */
void Detector3::displayResult(std::vector<bool> good_matching, int dimMax, bool wait)
{
  cv::Mat color = getResult(good_matching,dimMax);
  cv::imshow("Matching",color);
  if(wait) cv::waitKey(0);
  else cv::waitKey(10);
}

/** Save an image with all correspondences found
 * @param dimMax maximal width for the output image
 * @param filename is the file name with extension (default: "result.jpg")
 */
void Detector3::saveResult(std::string filename, int dimMax)
{
  cv::Mat color = getResult(dimMax);
  cv::imwrite(filename,color);
}

/** Save an image with all correspondences found
 * @param good_matching is input bool vector to specify if a pair of point is an inlier or outlier
 * @param dimMax maximal width for the output image
 * @param filename is the file name with extension (default: "result.jpg")
 */
void Detector3::saveResult(std::string filename, std::vector<bool> good_matching, int dimMax)
{
  cv::Mat color = getResult(good_matching,dimMax);
  cv::imwrite(filename,color);
}

/** Get the ith image
 * @param index is the image number (1, 2 or 3)
 * @return cv::Mat image
 */
cv::Mat Detector3::getImage(int index)
{
  assert(index>=1 && index <=3);
  switch(index){
  case 1 : return img1;
  case 2 : return img2;
  case 3 : return img3;
  default: return cv::Mat();
  }
}

/** Permute images, 1<-2 and 2<-3 if necessary\n
 * also permute descriptors
 */
void Detector3::permute()
{
  pts1_all = pts2_all;
  pts2_all = pts3_all;
  img1 = img2.clone();
  img2 = img3.clone();
  descriptors_img1 = descriptors_img2.clone();
  descriptors_img2 = descriptors_img3.clone();
  pts3_all.clear();
}

/** Permute images, 1<-2 and 2<-3 if necessary for old surf function\n
 * also permute descriptors
 */
void Detector3::permute_old()
{
  pts1_all = pts2_all;
  pts2_all = pts3_all;

  img1 = img2.clone();
  img2 = img3.clone();

  if(keypoints_img1_old){
    cvClearSeq(keypoints_img1_old);
    cvClearSeq(descriptors_img1_old);
  }
  if(keypoints_img2_old){
    keypoints_img1_old = cvCloneSeq(keypoints_img2_old);
    descriptors_img1_old = cvCloneSeq(descriptors_img2_old);
    cvClearSeq(keypoints_img2_old);
    cvClearSeq(descriptors_img2_old);
  }
  if(keypoints_img3_old){
    keypoints_img2_old = cvCloneSeq(keypoints_img3_old);
    descriptors_img2_old = cvCloneSeq(descriptors_img3_old);
    cvClearSeq(keypoints_img3_old);
    cvClearSeq(descriptors_img3_old);
  }

  ptpairs1 = ptpairs2;
  pts3_all.clear();
}

/** Compare two point descriptors
 * @param d1, d2 are input descriptors
 * @param best is the input threshold
 * @param length is the input descriptor size (should be 4)
 * @retval total coast
 */
double Detector3::compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
  double total_cost = 0;
  assert( length % 4 == 0 );
  for( int i = 0; i < length; i += 4 ){
    double t0 = d1[i] - d2[i];
    double t1 = d1[i+1] - d2[i+1];
    double t2 = d1[i+2] - d2[i+2];
    double t3 = d1[i+3] - d2[i+3];
    total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
    if( total_cost > best )
      break;
  }
  return total_cost;
}

/** Search the nearest neighbor
 * @param vec is input descriptor under vector of floats
 * @param laplacian
 * @param model_keypoints is input keypoints of the first image
 * @param model_descriptors is input descriptors of the first image points
 * @retval nearest neighbor index
 */
int Detector3::naiveNearestNeighbor( const float* vec, int laplacian, const CvSeq* model_keypoints, const CvSeq* model_descriptors )
{
  int length = (int)(model_descriptors->elem_size/sizeof(float));
  int i, neighbor = -1;
  double d, dist1 = 1e6, dist2 = 1e6;
  CvSeqReader reader, kreader;
  cvStartReadSeq( model_keypoints, &kreader, 0 );
  cvStartReadSeq( model_descriptors, &reader, 0 );

  for( i = 0; i < model_descriptors->total; i++ ){
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* mvec = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    if( laplacian != kp->laplacian )
      continue;
    d = compareSURFDescriptors( vec, mvec, dist2, length );
    if( d < dist1 ){
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    }
    else if ( d < dist2 )
      dist2 = d;
  }
  if ( dist1 < 0.6*dist2 )
    return neighbor;
  return -1;
}

/** Find pairs using point descriptors
 * @param objectKeypoints,imageKeypoints are input keypoints
 * @param objectDescriptors,imageDescriptors are input descriptors
 * @param ptpairs is the output pairs vector
 */
void Detector3::findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors, const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, std::vector<int>& ptpairs )
{
  int i;
  CvSeqReader reader, kreader;
  cvStartReadSeq( objectKeypoints, &kreader );
  cvStartReadSeq( objectDescriptors, &reader );
  ptpairs.clear();

  for( i = 0; i < objectDescriptors->total; i++ ){
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
    if( nearest_neighbor >= 0 ){
      ptpairs.push_back(i);
      ptpairs.push_back(nearest_neighbor);
    }
  }
}


std::vector<cv::Point2d> Detector3::getSurfOld(cv::Mat I)
{
  cv::Mat Img;
  if(I.channels()==3)
    cv::cvtColor(I,Img,CV_RGB2GRAY);
  else
    Img = I;

  IplImage gray1 = Img;

  CvSURFParams params = cvSURFParams(_hessianThreshold, 1);
  cvExtractSURF( &gray1, 0, &keypoints_img1_old, &descriptors_img1_old, storage1, params );
  return extractPoints(keypoints_img1_old);
}

std::vector<cv::Point2d> Detector3::getSurf(cv::Mat I)
{
  cv::SURF detector(_hessianThreshold,_nOctaves,_nOctaveLayersSURF,_extended);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(I, keypoints);
  std::vector<cv::Point2d> pts;
  for(int i=0;i<keypoints.size();i++)
    pts.push_back( keypoints.at(i).pt );
  return pts;
}

std::vector<cv::Point2d> Detector3::getSift(cv::Mat I)
{
  cv::SIFT detector(_nfeatures, _nOctaveLayersSIFT, _contrastThreshold, _edgeThreshold, _sigma);
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(I, keypoints);
  std::vector<cv::Point2d> pts;
  for(int i=0;i<keypoints.size();i++)
    pts.push_back( keypoints.at(i).pt );
  return pts;
}

bool Detector3::getCorrespondingPoint(std::vector<cv::Point2d> &l1, std::vector<cv::Point2d> &l2, cv::Point2d &p1, cv::Point2d &p2)
{
  assert(l1.size()==l2.size());
  for(int i=0;i<l1.size();i++){
    if(l1.at(i)==p1){
      p2 = l2.at(i);
      return true;
    }
  }
  return -1;
}
