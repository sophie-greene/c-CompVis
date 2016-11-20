#include "../include/detector.h"

/** Load two images from files
 * @param file1,file2 are input filenames
 * @param flip flip input images (0=none, 1=first, 2=second, 3=both)
 */
Detector::Detector(std::string file1, std::string file2, int flip)
{
  img1 = cv::imread(file1.c_str());
  img2 = cv::imread(file2.c_str());
  flipImages(flip);
  init();
}

/** Load two images from arguments
 * @param im1,im2 are input images
 * @param flip flip input images (0=none, 1=first, 2=second, 3=both)
 */
Detector::Detector(cv::Mat im1, cv::Mat im2, int flip)
{
  img1 = im1;
  img2 = im2;
  flipImages(flip);
  init();
}

/** Create and init an empty Detector
 */
Detector::Detector()
{
  init();
}

/** Create and init an empty Matcher FeatureType/DescriptorType/MatcherType with types
 * @param featureType is the input FeatureType
 * @param descriptorType is the input DescriptorType
 * @param matcherType is the input MatcherType
 */
Detector::Detector(FeatureType featureType, DescriptorType descriptorType, MatcherType matcherType)
{
  init();
  setType(featureType,descriptorType,matcherType);
}

/** Destroy the object and release memory storage*/
Detector::~Detector()
{
  cvReleaseMemStorage(&storage1);
  cvReleaseMemStorage(&storage2);
}

/** Initialize variables
 */
void Detector::init()
{
  keypoints_img1_old = 0;
  keypoints_img2_old = 0;
  descriptors_img1_old = 0;
  descriptors_img2_old = 0;
  storage1 = cvCreateMemStorage();
  storage2 = cvCreateMemStorage();
  setType(NO_FEATURE_TYPE,NO_DESCRIPTOR_TYPE,NO_MATCHER_TYPE);
}

/** Flip images
 * @param mode flip images (in binary, ex: to flip 1&2 use 0b11 -> 3)
 */
void Detector::flipImages(int mode)
{
  if(mode==1 || mode==3)
    cv::flip(img1,img1,1);
  if(mode==2 || mode==3)
    cv::flip(img2,img2,1);
}

/** Set two images (images are cloned)
 * @param im1,im2 are input images
 */
void Detector::setImages(cv::Mat &im1, cv::Mat &im2)
{
  img1 = im1.clone();
  img2 = im2.clone();
}

/** Read to images from files
 * @param file1,file2 are input filenames
 */
void Detector::setImages(std::string file1, std::string file2)
{
  img1 = cv::imread(file1.c_str());
  img2 = cv::imread(file2.c_str());
}

/** Set Feature, Decriptor and Matcher type
 * @param featureType,descriptorType,matcherType are input type
 */
void Detector::setType(FeatureType featureType, DescriptorType descriptorType, MatcherType matcherType)
{
  currentFeatureType = featureType;
  currentDescriptorType = descriptorType;
  currentMatcherType = matcherType;
}

/** Get Feature type
 * @param type is feature name
 */
Detector::FeatureType Detector::getFeatureType(std::string type)
{
  if(type=="FAST") return FAST;
  if(type=="STAR") return STAR;
  if(type=="SIFT") return SIFT;
  if(type=="SURF") return SURF;
  if(type=="OLDSURF") return OLDSURF;
  if(type=="ORB") return ORB;
  if(type=="MSER") return MSER;
  if(type=="GFTT") return GFTT;
  if(type=="HARRIS") return HARRIS;
  if(type=="DENSE") return DENSE;
  if(type=="BLOB") return BLOB;
  return NO_FEATURE_TYPE;
}

/** Get Descriptor type
 * @param type is descriptor name
 */
Detector::DescriptorType Detector::getDescriptorType(std::string type)
{
  if(type=="SIFT") return SIFT_EXTRACTOR;
  if(type=="SURF") return SURF_EXTRACTOR;
  if(type=="ORB") return ORB_EXTRACTOR;
  if(type=="BRIEF") return BRIEF_EXTRACTOR;
  return NO_DESCRIPTOR_TYPE;
}

/** Get Matcher type
 * @param type is matcher name
 */
Detector::MatcherType Detector::getMatcherType(std::string type)
{
  if(type=="L1") return L1;
  if(type=="L2") return L2;
  if(type=="HAMMING1") return HAMMING1;
  if(type=="HAMMING2") return HAMMING2;
  if(type=="FLANN") return FLANN;
  return NO_MATCHER_TYPE;
}

/** Set default parameters
 * @param type is feature type
 */
void Detector::setDefaultParameters(FeatureType type)
{
  switch(type){
    case FAST: setParam(2,10,1); break;
    case STAR: setParam(5,45,30,10,8,5); break;
    case SIFT: setParam(5,0,3,0.04,10,1.6); break;
    case SURF: setParam(5,10.0,4,2,1,0); break;
    case ORB:  setParam(6,500,1.2,8,31,0,2,0,31); break;
    case MSER: setParam(9,5,60,1000,0.25,0.2,200,1.01,0.003,5); break;
    case GFTT: setParam(6,1000,0.01,1,3,0,0.04); break;
    case DENSE: setParam(7,1.0,1,0.1,6,0,1,0); break;
    case BLOB: setParam(0); break;

    case OLDSURF: setParam(2,100,1); break;
    case HARRIS: setParam(4,500,50,50,0.8); break;
  }
}

/** Display help to set parameters
 */
void Detector::displayParametersHelp()
{
    std::cout << "Use 1 for true, 0 for false" << std::endl;
    std::cout << "FAST : ( int threshold=1, bool nonmaxSuppression=true, type=FastFeatureDetector::TYPE_9_16 )" << std::endl;
    std::cout << "STAR : ( int maxSize=16, int responseThreshold=30, int lineThresholdProjected = 10, int lineThresholdBinarized=8, int suppressNonmaxSize=5 )" << std::endl;
    std::cout << "DENSE : ( float initFeatureScale=1.f, int featureScaleLevels=1, float featureScaleMul=0.1f, int initXyStep=6, int initImgBound=0, bool varyXyStepWithScale=true, bool varyImgBoundWithScale=false )" << std::endl;
    std::cout << "MSER : ( int delta=5, int minArea=30, int maxArea=1000, double maxVariation=0.25, double minDiversity=0.2, int maxEvolution=200, double areaThreshold=1.01, double minMargin=0.003, int edgeBlurSize=5 )" << std::endl;
    std::cout << "GFTT : ( int maxCorners, double qualityLevel, double minDistance, int blockSize=3, bool useHarrisDetector=false, double k=0.04 )" << std::endl;
    std::cout << "SURF : ( double hessianThreshold=10.0, int nOctaves=4, int nOctaveLayers=2, bool extended=true, bool upright=false )" << std::endl;
    std::cout << "SIFT : ( int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6 )" << std::endl;
    std::cout << "ORB : ( int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31, int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31 ) [HARRIS_SCORE=0, FAST_SCORE=1]" << std::endl;
    std::cout << "HARRIS : ( int nb_point_max=500, int win_width=50, win_height=50, double score=0.8 )  [From Eric Royer]" << std::endl;
    std::cout << "OLDSURF : (double hessianThreshold=100, int extended=1 )" << std::endl;
}

/** Set parameters
  @param nbParam is the number of parameters to set (<=10)
  @param p1,p2,p3,p4,p5,p6,p7,p8,p9,p10 are parameters value
  @warning Use 1 for 'true' and 0 for 'false'
 */
void Detector::setParam(int nbParam, double p1, double p2, double p3, double p4, double p5, double p6, double p7, double p8, double p9, double p10)
{
  if(nbParam>=1) parameters[0] = p1;
  if(nbParam>=2) parameters[1] = p2;
  if(nbParam>=3) parameters[2] = p3;
  if(nbParam>=4) parameters[3] = p4;
  if(nbParam>=5) parameters[4] = p5;
  if(nbParam>=6) parameters[5] = p6;
  if(nbParam>=7) parameters[6] = p7;
  if(nbParam>=8) parameters[7] = p8;
  if(nbParam>=9) parameters[8] = p9;
  if(nbParam>=10) parameters[9] = p10;
}

/** Set parameters
  @param params is the vector of parameters (size<10)
  @warning Use 1 for 'true' and 0 for 'false'
 */
void Detector::setParam(std::vector<double> params)
{
  assert(params.size()<10);
  for(int i=0;i<params.size();i++)
    parameters[i] = params.at(i);
}

/** Compute matching using input types
  @param featureType is the FeatureType
  @param descriptorType is the DescriptorType
  @param matcherType is the MatcherType
 */
void Detector::compute(FeatureType featureType, DescriptorType descriptorType, MatcherType matcherType)
{
  setType(featureType,descriptorType,matcherType);
  compute();
}

/** Compute matching
  */
void Detector::compute()
{
    assert(!img1.empty() && !img2.empty());

    assert(currentFeatureType!=NO_FEATURE_TYPE);
    if(currentFeatureType==HARRIS){
      computeHarris();
      return;
    }
    if(currentFeatureType==OLDSURF){
      computeSURF();
      return;
    }
    assert(currentDescriptorType!=NO_DESCRIPTOR_TYPE);
    assert(currentMatcherType!=NO_MATCHER_TYPE);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    cv::FeatureDetector *detector;
    switch(currentFeatureType){
      case FAST: detector = new cv::FastFeatureDetector(parameters[0],toBool(parameters[1])); break;
      case STAR: detector = new cv::StarDetector(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4]); break;
      case SIFT: detector = new cv::SIFT(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4]); break;
      case SURF: detector = new cv::SURF(parameters[0],parameters[1],parameters[2],toBool(parameters[3]),toBool(parameters[4])); break;
      case ORB:  detector = new cv::ORB(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5]); break;
      case MSER: detector = new cv::MSER(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5]); break;
      case GFTT: detector = new cv::GFTTDetector(parameters[0],parameters[1],parameters[2],parameters[3],toBool(parameters[4]),parameters[5]); break;
      case DENSE: detector = new cv::DenseFeatureDetector(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],toBool(parameters[5]),toBool(parameters[6])); break;
      case BLOB: detector = new cv::SimpleBlobDetector(); break;
    }
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);

    // computing descriptors
    cv::Mat descriptors1, descriptors2;

    cv::DescriptorExtractor *extractor;
    switch(currentDescriptorType)
    {
      case SIFT_EXTRACTOR: extractor = new cv::SiftDescriptorExtractor; break;
      case SURF_EXTRACTOR: extractor = new cv::SurfDescriptorExtractor; break;
      case ORB_EXTRACTOR: extractor = new cv::OrbDescriptorExtractor; break;
      case BRIEF_EXTRACTOR: extractor = new cv::BriefDescriptorExtractor(); break;
    }

    extractor->compute(img1, keypoints1, descriptors1);
    extractor->compute(img2, keypoints2, descriptors2);

    // matching descriptors
    std::vector< cv::DMatch > matches;
    cv::DescriptorMatcher *matcher;

    switch(currentMatcherType)
    {
      case L1: matcher = new cv::BFMatcher(cv::NORM_L1); break;
      case L2: matcher = new cv::BFMatcher(cv::NORM_L2); break;
      case HAMMING1: matcher = new cv::BFMatcher(cv::NORM_HAMMING); break;
      case HAMMING2: matcher = new cv::BFMatcher(cv::NORM_HAMMING2); break;
      case FLANN: matcher = new cv::FlannBasedMatcher(); break;
    }

    matcher->match(descriptors1, descriptors2, matches);

    // save points
    pts1_all.clear();
    pts2_all.clear();
    for( unsigned int m = 0; m < keypoints1.size(); m++ ){
      cv::KeyPoint kp1 = keypoints1[m];
      cv::Point2f pt1(kp1.pt);
      pts1_all.push_back(pt1);
    }
    for( unsigned int m = 0; m < keypoints2.size(); m++ ){
      cv::KeyPoint kp2 = keypoints2[m];
      cv::Point2f pt2(kp2.pt);
      pts2_all.push_back(pt2);
    }
    // save matches
    pts1.clear();
    pts2.clear();
    correspondences.clear();
    for( unsigned int m = 0; m < matches.size(); m++ ){
      int i1 = matches.at(m).queryIdx;
      int i2 = matches.at(m).trainIdx;
      correspondences.push_back( std::make_pair(i1,i2) );
      cv::KeyPoint kp1 = keypoints1[i1], kp2 = keypoints2[i2];
      cv::Point2f pt1(kp1.pt);
      cv::Point2f pt2(kp2.pt);
      pts1.push_back(pt1);
      pts2.push_back(pt2);
    }
}

/** Compute matching using Eric Royer library
  */
void Detector::computeHarris()
{
  cv::Mat I1 = img1.clone();
  cv::Mat I2 = img2.clone();
  if(I1.channels()==3) cv::cvtColor(I1, I1, CV_RGB2GRAY);
  if(I2.channels()==3) cv::cvtColor(I2, I2, CV_RGB2GRAY);
  local_vis_eric::MCharImage Image1, Image2;
  Image1.ConvertFrom(I1.data,I1.cols,I1.rows);
  Image2.ConvertFrom(I2.data,I2.cols,I2.rows);

  int nb_point_max = parameters[0];
  int win_width = parameters[1];
  int win_height = parameters[2];
  double score = parameters[3];

  local_vis_eric::MDetecteurHarris Detecteur1(Image1);
  local_vis_eric::MDetecteurHarris Detecteur2(Image2);

  local_vis_eric::MListePI ListePI1,ListePI2;
  Detecteur1.Detecte(Image1,ListePI1,nb_point_max);
  Detecteur2.Detecte(Image2,ListePI2,nb_point_max);

  local_vis_eric::MListeCouples ListeCouples;
  ListeCouples.Apparie(ListePI1,Detecteur1,ListePI2,Detecteur2,win_width,win_height,score);

  pts1.clear();
  pts2.clear();
  for(int i=0;i<ListeCouples.getNbCouples();i++){
    float x1,y1,x2,y2;
    ListeCouples.GetCoords(i,x1,y1,x2,y2);
    pts1.push_back( cv::Point2d(x1,y1) );
    pts2.push_back( cv::Point2d(x2,y2) );
  }
}

/** Compute matching using SURF from old OpenCV functions
 */
void Detector::computeSURF()
{
  cv::Mat I1,I2;
  if(img1.channels()==3) cv::cvtColor(img1,I1,CV_RGB2GRAY);
  else I1 = img1;

  if(img2.channels()==3) cv::cvtColor(img2,I2,CV_RGB2GRAY);
  else I2 = img2;

  IplImage gray1 = I1, gray2 = I2;

  CvSURFParams params = cvSURFParams(parameters[0], parameters[1]);

  cvExtractSURF( &gray1, 0, &keypoints_img1_old, &descriptors_img1_old, storage1, params );
  pts1_all = extractPoints(keypoints_img1_old);

  cvExtractSURF( &gray2, 0, &keypoints_img2_old, &descriptors_img2_old, storage2, params );
  pts2_all = extractPoints(keypoints_img2_old);

  ptpairs1.clear();
  findPairs( keypoints_img1_old, descriptors_img1_old, keypoints_img2_old, descriptors_img2_old, ptpairs1 );

  pts1.clear();
  pts2.clear();
  correspondences.clear();
  for(int i = 0; i < (int)ptpairs1.size(); i += 2 ) {
    int index1 = ptpairs1[i];
    int index2 = ptpairs1[i+1];
    correspondences.push_back( std::make_pair(index1,index2));
    CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( keypoints_img1_old, index1 );
    CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( keypoints_img2_old, index2 );
    cv::Point2d pt1(r1->pt);
    cv::Point2d pt2(r2->pt);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }
}

/** Extract points from a sequence
 * @param seq is the input sequence
 * @retval vector of points
 */
std::vector<cv::Point2d> Detector::extractPoints(CvSeq* seq)
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
std::vector<cv::Point2d> Detector::getPoints(int index)
{
  assert(index>=1 && index <=2);
  switch(index){
  case 1 : return pts1;
  case 2 : return pts2;
  default: return std::vector<cv::Point2d>();
  }
}

/** Return points in image 'index'
 * @param index image index [1:3]
 * @retval points vector in the image
 */
std::vector<cv::Point2d> Detector::getAllPoints(int index)
{
  assert(index>=1 && index <=2);
  switch(index){
  case 1 : return pts1_all;
  case 2 : return pts2_all;
  default: return std::vector<cv::Point2d>();
  }
}

/** Return correspondences between image 'index' and inmage 'index+1'
 * @param index image index [1:2]
 * @retval correspondences vector
 */
std::vector< std::pair<int,int> > Detector::getCorrespondences()
{
  return correspondences;
}

/** Return couples using Eric Royer list
 */
local_vis_eric::MListeCouples Detector::getCouples()
{
  local_vis_eric::MListeCouples list;
  for(int i=0;i<getNbPairs();i++)
    list.Add((float)pts1.at(i).x,(float)pts1.at(i).y, (float)pts2.at(i).x,(float)pts2.at(i).y,0.0);
  return list;
}

/** Set points in image 'index'
 * @param pts input list of 2D points
 * @param index image index [1:3]
 */
void Detector::setPoints(std::vector<cv::Point2d> pts, int index)
{
  assert(index>=1 && index <=2);
  switch(index){
  case 1 : pts1 = pts; break;
  case 2 : pts2 = pts; break;
  }
}

/** Return the number of correspondences
 * @retval min of pts1 and pts2 length
 */
int Detector::getNbPairs()
{
  return MIN(pts1.size(),pts2.size());
}

/** Get an image with all correspondences found
 * @param dimMax maximal width for the output image
 * @param displayAll set to true to draw inliers AND outliers
 */
cv::Mat Detector::getResult(int dimMax, bool displayAll)
{
  std::vector<bool> good_matching;
  return getResult(good_matching,dimMax,displayAll);
}

/** Get an image with all correspondences found
 * @param good_matching is input bool vector to specify if a pair of point is an inlier or outlier
 * @param dimMax maximal width for the output image
 * @param displayAll set to true to draw inliers AND outliers
 */
cv::Mat Detector::getResult(std::vector<bool> good_matching, int dimMax, bool displayAll)
{
  cv::Mat color;
  cv::Size size( img1.cols + img2.cols, MAX(img1.rows, img2.rows) );

  color.create( size, CV_MAKETYPE(img1.depth(), 3) );

  cv::Mat outImg1, outImg2;
  outImg1 = color( cv::Rect(0, 0, img1.cols, img1.rows) );
  outImg2 = color( cv::Rect(img1.cols, 0, img2.cols, img2.rows) );

  if(img1.channels()==3){
    img1.copyTo(outImg1);
  }else{
    cv::Mat I1;
    cv::cvtColor(img1,I1,CV_GRAY2RGB);
    I1.copyTo(outImg1);
  }
  if(img2.channels()==3){
    img2.copyTo(outImg2);
  }else{
    cv::Mat I2;
    cv::cvtColor(img2,I2,CV_GRAY2RGB);
    I2.copyTo(outImg2);
  }

  for(unsigned int i=0;i<pts1.size();i++){
    cv::Point2d pt1 = pts1.at(i);
    cv::Point2d pt2 = pts2.at(i);
    pt2.x += img1.cols;
    cv::Scalar linecolor = CV_RGB(255,255,255);
    bool draw = true;
    if(i<good_matching.size()){
      if(good_matching.at(i)){
        linecolor = CV_RGB(0,255,0);
      }else{
        linecolor = CV_RGB(255,0,0);
        if(!displayAll)
          draw = false;
      }
    }else{
      if(good_matching.size()==0)
       linecolor = CV_RGB(rand()%255,rand()%255,rand()%255);
    }
    if(draw){
      cv::circle( color, pt1, 3, CV_RGB(0,0,255), -1);
      cv::circle( color, pt2, 3, CV_RGB(0,0,255), -1);
      cv::line( color, pt1, pt2, linecolor, 1);
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
void Detector::displayResult(int dimMax, bool wait)
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
void Detector::displayResult(std::vector<bool> good_matching, int dimMax, bool wait)
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
void Detector::saveResult(std::string filename, int dimMax)
{
  cv::Mat color = getResult(dimMax);
  cv::imwrite(filename,color);
}

/** Save an image with all correspondences found
 * @param good_matching is input bool vector to specify if a pair of point is an inlier or outlier
 * @param dimMax maximal width for the output image
 * @param filename is the file name with extension (default: "result.jpg")
 */
void Detector::saveResult(std::string filename, std::vector<bool> good_matching, int dimMax)
{
  cv::Mat color = getResult(good_matching,dimMax);
  cv::imwrite(filename,color);
}

/** Get the ith image
 * @param index is the image number (1, 2)
 * @return cv::Mat image
 */
cv::Mat Detector::getImage(int index)
{
  assert(index>=1 && index <=2);
  switch(index){
  case 1 : return img1;
  case 2 : return img2;
  default: return cv::Mat();
  }
}

/** Compare two point descriptors
 * @param d1, d2 are input descriptors
 * @param best is the input threshold
 * @param length is the input descriptor size (should be 4)
 * @retval total coast
 */
double Detector::compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
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
int Detector::naiveNearestNeighbor( const float* vec, int laplacian, const CvSeq* model_keypoints, const CvSeq* model_descriptors )
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
void Detector::findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors, const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, std::vector<int>& ptpairs )
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

/** Extract points from an image using setted types
  @param I is the input image
  */
std::vector<cv::Point2d> Detector::getPoints(cv::Mat I)
{
  assert(currentFeatureType!=NO_FEATURE_TYPE);
  if(currentFeatureType==HARRIS){
    return getPointsHarris(I);
  }
  if(currentFeatureType==OLDSURF){
    return getPointsSURF(I);
  }

  std::vector<cv::KeyPoint> keypoints;

  cv::FeatureDetector *detector;
  switch(currentFeatureType){
    case FAST: detector = new cv::FastFeatureDetector(parameters[0],toBool(parameters[1])); break;
    case STAR: detector = new cv::StarDetector(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4]); break;
    case SIFT: detector = new cv::SIFT(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4]); break;
    case SURF: detector = new cv::SURF(parameters[0],parameters[1],parameters[2],toBool(parameters[3]),toBool(parameters[4])); break;
    case ORB:  detector = new cv::ORB(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5]); break;
    case MSER: detector = new cv::MSER(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],parameters[5]); break;
    case GFTT: detector = new cv::GFTTDetector(parameters[0],parameters[1],parameters[2],parameters[3],toBool(parameters[4]),parameters[5]); break;
    case DENSE: detector = new cv::DenseFeatureDetector(parameters[0],parameters[1],parameters[2],parameters[3],parameters[4],toBool(parameters[5]),toBool(parameters[6])); break;
    case BLOB: detector = new cv::SimpleBlobDetector(); break;
  }
  detector->detect(I, keypoints);

  std::vector<cv::Point2d> pts;
  for(int i=0;i<keypoints.size();i++)
    pts.push_back( cv::Point2f(keypoints.at(i).pt ) );
  return pts;
}

/** Extract points from an image using Eric Royer library
  @param I is the input image
  */
std::vector<cv::Point2d> Detector::getPointsHarris(cv::Mat I)
{
  if(I.channels()==3) cv::cvtColor(I, I, CV_RGB2GRAY);
  local_vis_eric::MCharImage Image1;
  Image1.ConvertFrom(I.data,I.cols,I.rows);

  int nb_point_max = parameters[0];

  local_vis_eric::MDetecteurHarris Detecteur1(Image1);

  local_vis_eric::MListePI ListePI1;
  Detecteur1.Detecte(Image1,ListePI1,nb_point_max);

  std::vector<cv::Point2d> pts;
  for(int i=0;i<ListePI1.getNbPoints();i++){
    double x = ListePI1.getX(i);
    double y = ListePI1.getY(i);
    pts.push_back( cv::Point2d(x,y) );
  }
  return pts;
}

/** Extract points from an image using old OpenCV SURF function
  @param I is the input image
  */
std::vector<cv::Point2d> Detector::getPointsSURF(cv::Mat I)
{
  cv::Mat Img;
  if(I.channels()==3)
    cv::cvtColor(I,Img,CV_RGB2GRAY);
  else
    Img = I;

  IplImage gray = Img;

  CvSURFParams params = cvSURFParams(parameters[0], parameters[1]);
  CvSeq *keypoints = 0, *descriptors = 0;
  CvMemStorage *storage  = cvCreateMemStorage();;
  cvExtractSURF( &gray, 0, &keypoints, &descriptors, storage, params );

  std::vector<cv::Point2d> pts = extractPoints(keypoints);
  cvReleaseMemStorage(&storage);

  return pts;
}
